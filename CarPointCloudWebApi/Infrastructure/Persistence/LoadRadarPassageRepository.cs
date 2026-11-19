using Dm;

namespace CarPointCloudWebApi.Infrastructure;

public sealed partial class LoadRadarRepository : ILoadRadarPassageRepository
{
    public LoadPassageStartRecord RegisterPassageStart(LoadPassageStartRecord record, VehicleArrivalRecord arrival,
        int registrationMaxSeconds)
    {
        for (var attempt = 0; ; attempt++)
        {
            try
            {
                return Transaction((connection, transaction) =>
                {
                    // 与识牌完成使用同一 camera 锚点；锁等待后重新检查登记期限，不能晚补旧邻居。
                    IntegrationRepository.LockCameraAnchor(connection, transaction, record.CameraIdentifier, required: false);
                    var existing = ReadPassageStart(connection, transaction, record.PassageKey);
                    var workflow = IntegrationRepository.ReadVehicleArrival(connection, transaction, record.WorkflowId, _timeout);
                    if (existing is not null)
                    {
                        if (workflow is null) throw Conflict("原到车登记不完整，保留观测待核查，不能补造旧识牌窗口");
                        return SameStart(existing, record);
                    }
                    if (workflow is not null) throw Conflict("原 workflow 已存在但缺少到车上下文");
                    CheckRegistrationTime(record, registrationMaxSeconds);
                    var stored = InsertPassageStart(connection, transaction, record);
                    IntegrationRepository.InsertVehicleArrival(connection, transaction, arrival, _timeout);
                    CheckRegistrationTime(record, registrationMaxSeconds);
                    return stored;
                });
            }
            catch (DmException ex) when (ex.Number == -6602 && attempt < 2)
            {
                // 整个事务已回滚；并发重试必须重新核验 Start 和 VA 均完整。
            }
        }
    }

    private static void CheckRegistrationTime(LoadPassageStartRecord record, int seconds)
    {
        if (DateTimeOffset.UtcNow - record.Start.ArrivedAt > TimeSpan.FromSeconds(Math.Clamp(seconds, 0, 30)))
            throw Conflict("到车登记已超时，不能为旧车辆新建识牌窗口");
    }

    private LoadPassageStartRecord InsertPassageStart(DmConnection connection, DmTransaction transaction, LoadPassageStartRecord record)
    {
        using (var create = Command(connection, transaction,
            "INSERT INTO LoadRadarPassage (PassageKey, NodeId, ChannelId, PassageId) VALUES (?, ?, ?, ?)"))
        {
            V(create, "key", record.PassageKey, 64);
            V(create, "node", record.Start.NodeId, 64);
            V(create, "channel", record.Start.ChannelId, 64);
            V(create, "passage", record.Start.PassageId, 128);
            create.ExecuteNonQuery();
        }
        using var insert = Command(connection, transaction,
            "INSERT INTO LoadRadarPassageStart (PassageKey, PayloadJson) VALUES (?, ?)");
        V(insert, "key", record.PassageKey, 64);
        J(insert, "payload", record with { Binding = null });
        insert.ExecuteNonQuery();
        return record with { Binding = null };
    }

    public LoadPassageStartRecord? GetPassageStart(string key)
    {
        using var connection = Open();
        return ReadPassageStart(connection, null, key);
    }

    private LoadPassageStartRecord? ReadPassageStart(DmConnection connection, DmTransaction? transaction, string key)
    {
        using var command = Command(connection, transaction,
            "SELECT s.PayloadJson, p.BindingJson FROM LoadRadarPassageStart s JOIN LoadRadarPassage p ON p.PassageKey=s.PassageKey WHERE s.PassageKey = ?");
        V(command, "key", key, 64);
        using var reader = command.ExecuteReader();
        if (!reader.Read()) return null;
        return Decode<LoadPassageStartRecord>(reader.GetString(0)) with
        { Binding = reader.IsDBNull(1) ? null : Decode<LoadPassageBinding>(reader.GetString(1)) };
    }

    private static LoadPassageStartRecord SameStart(LoadPassageStartRecord old, LoadPassageStartRecord incoming)
    {
        if (old.Start != incoming.Start || old.PayloadHash != incoming.PayloadHash ||
            old.CameraIdentifier != incoming.CameraIdentifier || old.WorkflowId != incoming.WorkflowId)
            throw Conflict("相同车次的到车上下文或点位发生变化");
        return old;
    }

    public LoadPassageBinding BindStartedPassage(LoadPassageStartRecord record, LoadPassageBinding binding)
    {
        try
        {
            return Transaction((connection, transaction) =>
            {
                var authoritative = LockWorkflow(connection, transaction, record.WorkflowId);
                LockPassage(connection, transaction, record.PassageKey);
                var stored = ReadPassageStart(connection, transaction, record.PassageKey)
                    ?? throw new LoadRadarException(404, "到车上下文不存在");
                SameStart(stored, record);
                if (binding.PassageKey != stored.PassageKey || binding.WorkflowId != stored.WorkflowId ||
                    binding.CameraIdentifier != stored.CameraIdentifier || authoritative.Camera != binding.CameraIdentifier ||
                    authoritative.Plate != binding.LicensePlate || binding.VehicleIdentity != "plate:" + authoritative.Plate)
                    throw Conflict("到车上下文与权威车牌身份不一致");
                if (stored.Binding is { } prior)
                {
                    if (SameBinding(prior, binding)) return prior;
                    throw Conflict("本次车牌关联已固定，不能覆盖");
                }
                using var update = Command(connection, transaction,
                    "UPDATE LoadRadarPassage SET WorkflowId=?, LicensePlate=?, BindingJson=? WHERE PassageKey=? AND WorkflowId IS NULL");
                V(update, "workflow", binding.WorkflowId, 128);
                V(update, "plate", binding.LicensePlate, 32);
                J(update, "binding", binding);
                V(update, "key", binding.PassageKey, 64);
                if (update.ExecuteNonQuery() != 1) throw Conflict("本次车次已被其他关联占用");
                Audit(connection, transaction, "bind", binding.PassageKey, binding.Actor, binding.Reason, binding);
                return binding;
            });
        }
        catch (DmException ex) when (ex.Number == -6602)
        {
            var prior = GetPassageStart(record.PassageKey)?.Binding;
            if (prior is not null && SameBinding(prior, binding)) return prior;
            throw Conflict("本次 workflow 已被其他车次占用");
        }
    }
}
