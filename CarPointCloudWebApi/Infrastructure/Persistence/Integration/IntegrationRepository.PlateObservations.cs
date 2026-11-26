using System.Security.Cryptography;
using System.Text;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

public partial class IntegrationRepository
{
    private sealed record PlateObservation(DateTime FirstSeenAt, string CaptureTime, string Plate, bool Conflict);

    private static string ObservationKey(string camera, string enterId) =>
        Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(camera + "\0" + enterId)));

    private Dictionary<string, PlateObservation> ReadPlateObservations(DmConnection connection, DmTransaction tx,
        IEnumerable<string> keys)
    {
        var results = new Dictionary<string, PlateObservation>();
        foreach (var batch in keys.Distinct().Chunk(100))
        {
            var parameters = string.Join(",", batch.Select(_ => "?"));
            using var read = new DmCommand("SELECT ObservationKey,FirstSeenAt,CaptureTimeText,LicensePlate,IdentityConflict " +
                $"FROM PartnerPlateObservation WHERE ObservationKey IN ({parameters})", connection, tx)
                { CommandTimeout = _commandTimeoutSeconds };
            for (var i = 0; i < batch.Length; i++)
                AddVarcharParam(read, "@Key" + i, 64, batch[i], required: true);
            using var reader = read.ExecuteReader();
            while (reader.Read())
                results.Add(reader.GetString(0), new(reader.GetDateTime(1), reader.GetString(2),
                    reader.GetString(3), reader.GetInt32(4) != 0));
        }
        return results;
    }

    private void RefreshCandidateObservations(DmConnection conn, DmTransaction tx, string camera,
        IReadOnlyList<LicensePlateRecord> candidates)
    {
        // 查端返回后到认领前，另一响应可能已标记身份冲突。持同点位锁重新读再绑定。
        var groups = candidates.Where(c => !string.IsNullOrWhiteSpace(c.EnterId))
            .GroupBy(c => ObservationKey(camera, c.EnterId!)).ToArray();
        var rows = ReadPlateObservations(conn, tx, groups.Select(g => g.Key));
        foreach (var group in groups)
        {
            rows.TryGetValue(group.Key, out var observation);
            foreach (var candidate in group)
            {
                candidate.FirstSeenAt = observation?.FirstSeenAt;
                candidate.ConflictingIdentity = observation is null || observation.Conflict ||
                    observation.Plate != candidate.LicensePlate ||
                    !PartnerContract.TryParseTime(observation.CaptureTime, out var captured) || captured != candidate.CaptureTime;
            }
        }
    }

    public void ObservePartnerCaptures(string camera, IReadOnlyList<PartnerLicensePlateData> candidates, DateTime receivedAt)
    {
        // enterID 为原始字符串，可含前导零且没有合同长度上限。定长摘要只用来索引。
        var groups = candidates.GroupBy(c => ObservationKey(camera, c.EnterId)).OrderBy(g => g.Key).ToArray();
        if (groups.Length == 0) return;
        using var connection = CreateConnection();
        connection.Open();
        for (var attempt = 0; ; attempt++)
        {
            using var tx = (DmTransaction)connection.BeginTransaction();
            try
            {
                LockCameraAnchor(connection, tx, camera, required: false);
                // 历史抓拍在每次轮询重复出现。成批读取，只写新观察、冲突或更早收到的回执。
                var prior = ReadPlateObservations(connection, tx, groups.Select(g => g.Key));
                foreach (var group in groups)
                {
                    var first = group.First();
                    prior.TryGetValue(group.Key, out var existing);
                    var at = existing is not null && existing.FirstSeenAt < receivedAt ? existing.FirstSeenAt : receivedAt;
                    var conflict = existing?.Conflict == true || group.Any(c =>
                        c.CaptureTime != (existing?.CaptureTime ?? first.CaptureTime) ||
                        c.LicensePlate != (existing?.Plate ?? first.LicensePlate));
                    if (existing is null)
                    {
                        using var insert = new DmCommand(
                            "INSERT INTO PartnerPlateObservation(ObservationKey,CameraIdentifier,EnterId,FirstSeenAt,CaptureTimeText,LicensePlate,IdentityConflict) " +
                            "VALUES(?,?,?,?,?,?,?)", connection, tx) { CommandTimeout = _commandTimeoutSeconds };
                        AddVarcharParam(insert, "@Key", 64, group.Key, required: true);
                        AddVarcharParam(insert, "@Camera", 64, camera, required: true);
                        insert.Parameters.Add("@EnterId", DmDbType.Clob).Value = first.EnterId;
                        insert.Parameters.Add("@Seen", DmDbType.DateTime).Value = at;
                        AddVarcharParam(insert, "@Captured", 19, first.CaptureTime, required: true);
                        AddVarcharParam(insert, "@Plate", 32, first.LicensePlate, required: true);
                        insert.Parameters.Add("@Conflict", DmDbType.Int32).Value = conflict ? 1 : 0;
                        insert.ExecuteNonQuery();
                    }
                    else if (at < existing.FirstSeenAt || conflict != existing.Conflict)
                    {
                        using var update = new DmCommand(
                            "UPDATE PartnerPlateObservation SET FirstSeenAt=LEAST(FirstSeenAt,?), " +
                            "IdentityConflict=GREATEST(IdentityConflict,?) WHERE ObservationKey=?", connection, tx)
                            { CommandTimeout = _commandTimeoutSeconds };
                        update.Parameters.Add("@Seen", DmDbType.DateTime).Value = at;
                        update.Parameters.Add("@Conflict", DmDbType.Int32).Value = conflict ? 1 : 0;
                        AddVarcharParam(update, "@Key", 64, group.Key, required: true);
                        update.ExecuteNonQuery();
                    }
                    foreach (var item in group)
                    {
                        item.FirstSeenAt = at;
                        item.ConflictingIdentity = conflict;
                    }
                }
                tx.Commit();
                return;
            }
            catch (DmException ex) when (attempt == 0 && IsUniqueConstraintViolation(ex))
            {
                // 尚无来车锚点时，并发的人工首查可能先插入相同观察；重读赢家。
                tx.Rollback();
            }
        }
    }
}
