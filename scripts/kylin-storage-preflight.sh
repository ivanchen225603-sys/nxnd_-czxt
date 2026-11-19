#!/usr/bin/env bash
# 101 服务器磁盘只读预检：不删除文件、不停止服务、不执行数据库语句。
set -u
set -o pipefail
printf '\n== 时间与磁盘 ==\n'
date --iso-8601=seconds
df -hT
df -i
lsblk -o NAME,SIZE,FSTYPE,MOUNTPOINT
printf '\n== 实际挂载来源 ==\n'
for target in / /opt/carpointcloud /data/carpointcloud /dm8; do
    if [ -e "$target" ]; then
        printf '\n路径: %s\n' "$target"
        findmnt -T "$target" -o TARGET,SOURCE,FSTYPE,UUID,MAJ:MIN
    fi
done
printf '\n== 主要目录占用（不跨文件系统，每项最多30秒；子挂载需另行统计） ==\n'
for target in /opt/carpointcloud /data/carpointcloud /home/yxzk/cpc-release-20261029 /var/log; do
    if [ -d "$target" ]; then
        timeout 30 du -x -h --max-depth=2 "$target" | sort -h
        result=${PIPESTATUS[0]}
        if [ "$result" -ne 0 ]; then
            printf '统计未完整：%s，退出码=%s（124表示超时；其他错误请看上方信息）。\n' "$target" "$result"
        fi
    fi
done
printf '\n== 已删除但仍被进程占用的文件 ==\n'
if command -v lsof >/dev/null 2>&1; then
    timeout 15 lsof -nP +L1
    result=$?
    if [ "$result" -ne 0 ]; then
        printf 'lsof退出码=%s；可能无匹配文件、权限不足或超时，请结合上方信息，不能据此确认无占用。\n' "$result"
    fi
else
    printf '未安装 lsof，未执行此项。\n'
fi
printf '\n== 当前服务（不输出环境变量或私密配置） ==\n'
for unit in car-pointcloud-ladar-worker.service carpointcloud-analyzer.service carpointcloudwebapi.service carpointcloud-load-radar.service; do
    systemctl show "$unit" -p Id -p ActiveState -p SubState -p MainPID -p WorkingDirectory -p RequiresMountsFor --no-pager
done
printf '\n预检完成：以上操作未删除数据、重启服务或操作数据库。\n'
