(function () {
    'use strict';

    document.addEventListener('DOMContentLoaded', () => {
        const byId = id => document.getElementById(id);
        const form = byId('volumeQueryForm');
        const error = byId('queryError');
        let filters;
        let page = 1;
        let pageSize = 10;
        let total = 0;
        let requestId = 0;

        function resetFilters() {
            const now = new Date();
            const today = `${now.getFullYear()}-${String(now.getMonth() + 1).padStart(2, '0')}-${String(now.getDate()).padStart(2, '0')}`;
            byId('startDate').value = today;
            byId('endDate').value = today;
            byId('licensePlate').value = '';
            byId('channelId').value = '';
        }

        function search() {
            if (!form.reportValidity()) return;
            if (byId('startDate').value > byId('endDate').value) {
                error.textContent = '开始日期不能晚于结束日期';
                error.hidden = false;
                return;
            }
            filters = {
                startDate: byId('startDate').value,
                endDate: byId('endDate').value,
                licensePlate: byId('licensePlate').value.trim().toUpperCase(),
                channelId: byId('channelId').value
            };
            page = 1;
            return load();
        }

        async function load() {
            const currentRequest = ++requestId;
            const tbody = byId('volumeTableBody');
            const empty = byId('volumeEmpty');
            error.hidden = true;
            tbody.replaceChildren();
            empty.hidden = false;
            empty.textContent = '正在加载体积记录…';
            byId('volumeResults').setAttribute('aria-busy', 'true');
            byId('previousPage').disabled = true;
            byId('nextPage').disabled = true;
            byId('pageInfo').textContent = '正在查询…';
            try {
                const query = new URLSearchParams({ ...filters, page: String(page), pageSize: String(pageSize) });
                const { data } = await API.get('/api/volume-query?' + query);
                // 快速重新查询时，只呈现最后一次请求的条件和结果。
                if (currentRequest !== requestId) return;
                total = data.total;
                page = data.page;
                for (const row of data.items) {
                    const tr = document.createElement('tr');
                    const time = new Date(row.collectedAt).toLocaleString('zh-CN', {
                        year: 'numeric', month: '2-digit', day: '2-digit',
                        hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false
                    });
                    for (const [index, value] of [time, row.licensePlate, row.scaleName,
                        row.volumeM3 == null ? '—' : Number(row.volumeM3).toFixed(1)].entries()) {
                        const td = document.createElement('td');
                        td.textContent = value;
                        if (index === 3) td.className = 'volume-number';
                        tr.appendChild(td);
                    }
                    tbody.appendChild(tr);
                }
                empty.hidden = data.items.length > 0;
                empty.textContent = '暂无符合条件的体积记录，请调整查询条件。';
                const pages = Math.max(1, Math.ceil(total / pageSize));
                byId('pageInfo').textContent = `共 ${total} 条 · 第 ${page}/${pages} 页`;
                byId('previousPage').disabled = page <= 1;
                byId('nextPage').disabled = page * pageSize >= total;
            } catch (e) {
                if (currentRequest !== requestId) return;
                empty.textContent = '查询未完成，请重试。';
                error.textContent = '查询失败：' + (e.message || '请稍后重试');
                error.hidden = false;
                byId('pageInfo').textContent = '查询未完成，记录数量未知';
            } finally {
                if (currentRequest === requestId) byId('volumeResults').setAttribute('aria-busy', 'false');
            }
        }

        form.addEventListener('submit', event => { event.preventDefault(); search(); });
        byId('resetButton').addEventListener('click', () => { resetFilters(); search(); });
        byId('previousPage').addEventListener('click', () => { page -= 1; load(); });
        byId('nextPage').addEventListener('click', () => { page += 1; load(); });
        byId('pageSize').addEventListener('change', () => {
            pageSize = Number(byId('pageSize').value);
            page = 1;
            load();
        });
        resetFilters();
        search();
    });
})();
