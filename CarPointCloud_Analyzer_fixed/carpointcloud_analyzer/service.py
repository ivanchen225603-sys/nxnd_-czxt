"""`python -m carpointcloud_analyzer.service`：启动 HTTP 分析服务（部署脚本使用的入口）。"""
from .composition import main

if __name__ == "__main__":
    raise SystemExit(main())
