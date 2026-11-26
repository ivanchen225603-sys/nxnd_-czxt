namespace CarPointCloudWebApi.Domain;

public enum PlateCaptureRelation { Compatible, Impossible, Uncertain }

/// <summary>在同一通道内，根据已确认的到磅和抓拍配对核对先后，不推断两锚之间的车辆数量。</summary>
public sealed class PlateCaptureOrder
{
    private readonly long[] _arrivals;
    private readonly long[] _captures;
    private readonly bool _conflicting;

    public PlateCaptureOrder(IEnumerable<(DateTime Arrival, DateTime Capture)> confirmedAnchors)
    {
        var anchors = confirmedAnchors.OrderBy(a => a.Arrival).ToArray();
        _arrivals = anchors.Select(a => Seconds(a.Arrival)).ToArray();
        _captures = anchors.Select(a => Seconds(a.Capture)).ToArray();
        for (var i = 1; i < anchors.Length; i++)
        {
            if (_arrivals[i] == _arrivals[i - 1] || _captures[i] <= _captures[i - 1])
            {
                _conflicting = true;
                break;
            }
        }
    }

    public PlateCaptureRelation Compare(DateTime arrival, DateTime capture)
    {
        if (_conflicting) return PlateCaptureRelation.Uncertain;
        var index = Array.BinarySearch(_arrivals, Seconds(arrival));
        var insertion = index < 0 ? ~index : index;
        var before = insertion - 1;
        var after = index < 0 ? insertion : index + 1;
        var captured = Seconds(capture);
        // 已确认配对严格同序。最靠近本车的一前一后就能约束所有更早或更晚的锚点。
        if ((before >= 0 && captured < _captures[before]) ||
            (after < _captures.Length && captured > _captures[after]))
            return PlateCaptureRelation.Impossible;
        if (index >= 0 || (before >= 0 && captured == _captures[before]) ||
            (after < _captures.Length && captured == _captures[after]))
            return PlateCaptureRelation.Uncertain;
        return PlateCaptureRelation.Compatible;
    }

    private static long Seconds(DateTime value) => value.Ticks / TimeSpan.TicksPerSecond;
}
