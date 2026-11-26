using System.Text.Json;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

public partial class IntegrationRepository
{
    public bool TryClaimStaticScan(long recordId, string workflowId, DateTime updatedAt, StaticCapture gate)
    {
        using var connection = CreateConnection();
        connection.Open();
        using var tx = (DmTransaction)connection.BeginTransaction();
        using var claim = new DmCommand(@"UPDATE VehicleArrival SET RadarStatus='PENDING', RadarError=NULL, UpdatedAt=?
            WHERE Id=? AND SessionId=? AND DeviceId='LOAD_RADAR' AND RadarStatus='NOT_REQUIRED'", connection, tx);
        claim.Parameters.Add("@At", DmDbType.DateTime).Value = updatedAt;
        claim.Parameters.Add("@Id", DmDbType.Int64).Value = recordId;
        AddVarcharParam(claim, "@Workflow", 128, workflowId);
        if (claim.ExecuteNonQuery() != 1) return false;
        // 自动派发标记由数据库触发器同事务建立，旧 WebApi 也不能绕过；漏装迁移则整笔回滚。
        WriteStaticScan(connection, tx, workflowId, gate);
        tx.Commit();
        return true;
    }

    public StaticDecision UpdateStaticScan(string workflowId, Action<StaticCapture> update)
    {
        using var connection = CreateConnection();
        connection.Open();
        using var tx = (DmTransaction)connection.BeginTransaction();
        using var read = new DmCommand("SELECT EvidenceJson FROM StaticCaptureEvidence WHERE WorkflowId=? FOR UPDATE",
            connection, tx);
        AddVarcharParam(read, "@Workflow", 128, workflowId);
        var json = read.ExecuteScalar() as string;
        if (json is null) throw new LoadRadarException(409, "该自动车次尚未登记静态证据");
        var gate = JsonSerializer.Deserialize<StaticCapture>(json) ??
            throw new InvalidDataException("静态证据记录损坏");
        update(gate);
        WriteStaticScan(connection, tx, workflowId, gate);
        tx.Commit();
        return gate.Decision;
    }

    private static void WriteStaticScan(DmConnection connection, DmTransaction tx, string workflowId, StaticCapture gate)
    {
        using var write = new DmCommand(
            "UPDATE StaticCaptureEvidence SET Decision=?, PairId=?, EvidenceJson=?, DeadlineMs=? WHERE WorkflowId=?", connection, tx);
        AddVarcharParam(write, "@Decision", 16, gate.Decision.Status);
        AddVarcharParam(write, "@Pair", 64, gate.Pair?.PairId);
        AddVarcharParam(write, "@Evidence", 4000, JsonSerializer.Serialize(gate));
        write.Parameters.Add("@Deadline", DmDbType.Int64).Value = gate.EvidenceDeadline.ToUnixTimeMilliseconds();
        AddVarcharParam(write, "@Workflow", 128, workflowId);
        if (write.ExecuteNonQuery() != 1) throw new InvalidOperationException("缺少自动采集标记，请核对 DM8 静态证据迁移");
    }

    public void ExpireStaticScans(DateTimeOffset now)
    {
        using var connection = CreateConnection();
        connection.Open();
        using var cmd = new DmCommand("SELECT TOP 100 WorkflowId FROM StaticCaptureEvidence " +
            "WHERE Decision='pending' AND DeadlineMs<=? ORDER BY DeadlineMs", connection);
        cmd.Parameters.Add("@Now", DmDbType.Int64).Value = now.ToUnixTimeMilliseconds();
        var due = new List<string>();
        using (var rows = cmd.ExecuteReader())
            while (rows.Read()) due.Add(rows.GetString(0));
        foreach (var id in due) UpdateStaticScan(id, gate => gate.Expire(now));
    }
}
