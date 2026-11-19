"""把显式物理模型接到本次真实测量；失败保留诊断，不伪造条件或底板参考。"""
from dataclasses import asdict

import numpy as np

from .bed_reference import BedReferenceScope, baseline_digest
from .cloud_observation import FrameSnapshot
from .detect import RegistrationDecision, estimate_registration
from .volume_inputs import align_extra_surfaces, surface_lineage


def runtime_volume_options(*, catalog, snapshot, baseline, loaded, frame, primary, support,
                           extra_members, extras, channel, capture_instance, epoch,
                           identity, configuration_hash):
    """只返回本次调用的参数与可冻结证据；不读配置、缓存、当前车次或数据库。"""
    registration = estimate_registration(baseline, loaded)
    evidence = dict(schema="volume-runtime-v1", status="unavailable", reason="缺少实际正式基线",
                    catalog_sha256=catalog.source_sha256, registration=asdict(registration))
    options = dict(dy=registration.dy_m, loaded_extra=None, volume_context=None, volume_scope=None)
    if baseline is None or loaded is None:
        return options, evidence
    try:
        aligned = align_extra_surfaces(loaded.source_rotation, extras)
        options["loaded_extra"] = aligned or None
        evidence["source_rotation"] = np.asarray(loaded.source_rotation).tolist()
        if snapshot is None or identity is None or snapshot.passage_id != identity.passage_id:
            raise ValueError("缺少与预留车次一致的正式基线快照")
        binding = snapshot.binding
        if (identity.channel != channel or binding["channel_id"] != channel or
                binding["configuration_sha256"] != configuration_hash or
                binding["baseline_sha256"] != baseline_digest(baseline)):
            raise ValueError("正式基线快照与实际配置或输入不一致")
        ids, members = surface_lineage(primary, support, extra_members)
        if (primary.pairs[-1].channel != channel or primary.pairs[-1].epoch != epoch or
                FrameSnapshot.freeze(frame) != FrameSnapshot.freeze(primary.input_frame())):
            raise ValueError("体积主输入与本次活动代次/观测窗口不一致")
        if len(extras) != len(extra_members) or any(not np.array_equal(extra,
                support.pairs[member].cropped.frame().xyz) for extra, member in zip(extras, extra_members)):
            raise ValueError("体积额外货面与实际观测成员不一致")
        # 主货面应是本次输入按实际矩阵旋转后的全部原行，不能用别的拟合结果。
        actual = align_extra_surfaces(loaded.source_rotation, (frame.xyz,))[0]
        if not np.array_equal(np.asarray(loaded.cloud, np.float64), actual):
            raise ValueError("主货面与本次输入的实际旋转不一致")
        scope = BedReferenceScope(binding["baseline_version_id"], binding["calibration_id"],
            configuration_hash, channel, identity.passage_id, capture_instance + ":" + str(epoch),
            ids, "baseline-heading-aligned-m-v1", frame.ts, members)
        evidence["scope"] = asdict(scope)
        evidence["binding"] = binding
        model, status = catalog.select(binding, frame.ts)
        if model is None:
            evidence.update(status=status, reason="本次正式基线未匹配到有效体积模型")
            return options, evidence
        value = model.data()
        evidence.update(model=value, model_sha256=model.sha256)
        if value["registration"]["mode"] == "fixed":
            registration = RegistrationDecision(float(value["registration"]["dy_m"]),
                "declared_fixed", "使用模型显式声明的固定偏移，非算法配准认证")
        evidence["registration"] = asdict(registration)
        options["dy"] = registration.dy_m
        context = model.supply(scope=scope, baseline=baseline, surfaces=(loaded.cloud,) + aligned,
                               dy=registration.dy_m, registration_status=registration.status)
        options.update(volume_context=context, volume_scope=scope)
        evidence.update(status="supplied", reason="显式模型条件已绑定实际输入，尚不代表现场认证",
                        context=asdict(context))
    except (ValueError, TypeError, KeyError, IndexError,
            OverflowError, FloatingPointError) as error:
        # ★ KeyError/IndexError 一并转 rejected（2026-09-23，低严重度批次）：
        #   数据缺字段（binding/channel_id、registration/mode 等）原来直接
        #   炸掉整次测量，靠外层兜底只剩异常类型名 —— 现在如实给出"缺哪个
        #   键"的拒绝理由，诊断线索不丢。
        reason = (str(error) if isinstance(error, (ValueError, TypeError))
                 else "%s: %s" % (type(error).__name__, error))
        evidence.update(status="rejected", reason=reason)
    return options, evidence
