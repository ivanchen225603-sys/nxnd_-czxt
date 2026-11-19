import logging
import os
import sys
from typing import Dict, List

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.application.pipeline import PointCloudAnalyzer
from carpointcloud_analyzer.domain.runtime import logger


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
)


# 默认 PCD 路径。命令行没给文件时回退到该环境变量，方便采集脚本直接调用。
DEFAULT_PCD_PATH = os.getenv('CARPOINT_PCD_PATH', '')


def parse_cli_args(argv: List[str]) -> Dict[str, str]:
    """
    解析命令行参数和环境变量。

    Args:
        argv: sys.argv 形式的命令行参数列表。
    Returns:
        dict: 标准化后的参数字典。baseline_dir 为废弃兼容参数，不进入算法。
    业务逻辑关键点：
        支持后台采集服务通过命令行传入 --camera、--plate；
        同时支持 CARPOINT_CAMERA_IDENTIFIER、CARPOINT_LICENSE_PLATE、
        CARPOINT_BASELINE_DIR、CARPOINT_PCD_PATH 环境变量。
    """
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('pcd_paths', nargs='*')
    parser.add_argument('--camera', default=os.getenv('CARPOINT_CAMERA_IDENTIFIER', ''))
    parser.add_argument('--plate', default=os.getenv('CARPOINT_LICENSE_PLATE', ''))
    parser.add_argument('--baseline-dir', default=os.getenv('CARPOINT_BASELINE_DIR'))
    args = parser.parse_args(argv[1:])
    pcd_paths = args.pcd_paths or ([DEFAULT_PCD_PATH] if DEFAULT_PCD_PATH else [])
    return {
        'pcd_path': pcd_paths[0] if pcd_paths else '',
        'pcd_paths': pcd_paths,
        'camera': args.camera,
        'plate': args.plate,
        'baseline_dir': args.baseline_dir,
    }


def main() -> int:
    """
    CLI 程序入口。

    Returns:
        int: 0 表示解析成功；1 表示文件不存在或解析异常。
    异常说明:
        main() 捕获顶层异常并记录日志，避免控制台调用时出现未格式化堆栈。
    """
    logger.info("=" * 60)
    logger.info("车辆点云结构分析（局部坐标路径）")
    logger.info("=" * 60)

    args = parse_cli_args(sys.argv)
    pcd_paths = args['pcd_paths']
    if not pcd_paths:
        logger.error(
            "未提供 PCD 文件路径。用法："
            "python -m carpointcloud_analyzer.cli <pcd文件路径> [更多pcd...]"
            "（或设置 CARPOINT_PCD_PATH 环境变量）"
        )
        _wait_exit()
        return 1
    for pcd_path in pcd_paths:
        if not os.path.exists(pcd_path):
            logger.error(f"文件不存在：{pcd_path}")
            _wait_exit()
            return 1

    try:
        cfg = Config()
        cfg.camera_identifier = args['camera']
        cfg.license_plate = args['plate']
        analyzer = PointCloudAnalyzer(cfg)
        all_outputs = []
        for index, pcd_path in enumerate(pcd_paths, 1):
            if len(pcd_paths) > 1:
                logger.info(f"批处理进度：{index}/{len(pcd_paths)} {pcd_path}")
            outputs = analyzer.analyze(pcd_path)
            all_outputs.append((pcd_path, outputs))
            logger.info("分析载荷已在内存中构造；不生成 PNG/JSON 文件")
        logger.info("=" * 60)
        logger.info(f"分析完成，共 {len(all_outputs)} 个文件")
        return 0
    except (FileNotFoundError, ValueError) as e:
        logger.error(f"输入错误：{e}")
        return 1
    finally:
        _wait_exit()


def _wait_exit() -> None:
    """在交互式终端中暂停，便于 Windows 双击运行时查看最终输出。

    Notes:
        重定向输入、服务模式或收到中断时直接返回，不影响进程退出码。
    """
    try:
        if sys.stdin.isatty():
            input("\n按回车退出...")
    except (EOFError, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    sys.exit(main())
