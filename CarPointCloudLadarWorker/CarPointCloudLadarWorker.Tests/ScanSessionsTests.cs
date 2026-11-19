namespace CarPointCloudLadarWorker.Tests;

public sealed class ScanSessionsTests
{
    private readonly TestTimeProvider _time = new();

    private ScanSessions Create(Action<LadarCaptureOptions>? configure = null) =>
        new(Microsoft.Extensions.Options.Options.Create(TestSupport.Options(configure)), _time, NullLogger<ScanSessions>.Instance);

    /// <summary>雷达约每 4 秒一场：推进时间后在回调入口取时间戳，与 RadarHost 一致。</summary>
    private string? Frame(ScanSessions sessions, string camera, int points = 1000, double afterSeconds = 4)
    {
        _time.AdvanceSeconds(afterSeconds);
        return sessions.Offer(camera, points, _time.GetTimestamp());
    }

    [Fact]
    public void Start_NewRequest_IsScanningAndSameRequestIsIdempotent()
    {
        var sessions = Create();

        var first = sessions.Start("wf-1");
        var again = sessions.Start("wf-1");

        Assert.Equal(("scanning", true, false), (first.Code, first.Started, first.Idempotent));
        Assert.Equal(ScanState.Scanning, first.Snapshot.State);
        Assert.Equal("wf-1", first.Snapshot.RequestId);
        Assert.Equal(("existing", false, true), (again.Code, again.Started, again.Idempotent));
        Assert.Equal(first.Snapshot.SessionId, again.Snapshot.SessionId);
    }

    [Fact]
    public void Start_WhileScanning_IsBusy()
    {
        var sessions = Create();
        sessions.Start("wf-1");

        var busy = sessions.Start("wf-2");

        Assert.Equal(("busy", false), (busy.Code, busy.Success));
        Assert.Equal("wf-1", busy.Snapshot.RequestId);
    }

    [Theory]
    [InlineData("bad\nid")]
    [InlineData("x")]
    public void Start_RejectsControlCharactersAndOverlongIds(string requestId)
    {
        var id = requestId == "x" ? new string('x', 129) : requestId;

        Assert.Equal("invalid_request_id", Create().Start(id).Code);
    }

    [Fact]
    public void Start_WithoutRequestId_GeneratesOne()
    {
        var result = Create().Start(null);

        Assert.True(result.Started);
        Assert.Equal(32, result.Snapshot.RequestId!.Length);
    }

    [Fact]
    public void Offer_DiscardsOneBoundaryFieldPerCameraThenAcceptsNext()
    {
        var sessions = Create();
        sessions.Start("wf-1");

        Assert.Null(Frame(sessions, "CAM_004"));
        Assert.Null(Frame(sessions, "CAM_005", afterSeconds: 0.05));
        Assert.Equal("wf-1", Frame(sessions, "CAM_004"));
        Assert.Null(Frame(sessions, "CAM_004", afterSeconds: 0.01));
        Assert.Equal("wf-1", Frame(sessions, "CAM_005", afterSeconds: 0.05));

        var snapshot = sessions.Snapshot();
        Assert.Equal((ScanState.Idle, ScanOutcome.Completed), (snapshot.State, snapshot.Outcome));
        Assert.Equal(["CAM_004", "CAM_005"], snapshot.CapturedCameras);
        Assert.Equal("pending", snapshot.PersistenceState);
    }

    [Fact]
    public void Offer_IgnoresFieldsInsideDeliveryGuardAndBeforeCommand()
    {
        var sessions = Create();
        var beforeCommand = _time.GetTimestamp();
        _time.AdvanceSeconds(0.1);
        sessions.Start("wf-1");

        Assert.Null(sessions.Offer("CAM_004", 1000, beforeCommand));
        Assert.Null(Frame(sessions, "CAM_004", afterSeconds: 1.9));   // 仍在 2s 交付窗内，不计边界
        Assert.Null(Frame(sessions, "CAM_004", afterSeconds: 0.2));   // 边界
        Assert.Equal("wf-1", Frame(sessions, "CAM_004"));
    }

    [Fact]
    public void Offer_IgnoresFieldsBelowMinimumPointCount()
    {
        var sessions = Create();
        sessions.Start("wf-1");

        Assert.Null(Frame(sessions, "CAM_004", points: 99));
        Assert.Null(Frame(sessions, "CAM_004"));   // 第一份合格场才是边界
        Assert.Equal("wf-1", Frame(sessions, "CAM_004"));
    }

    [Fact]
    public void Offer_WithoutSession_AcceptsNothing()
    {
        Assert.Null(Frame(Create(), "CAM_004"));
    }

    [Fact]
    public void Session_TimesOutWithMissingCameras()
    {
        var sessions = Create();
        sessions.Start("wf-1");
        Frame(sessions, "CAM_004");
        Frame(sessions, "CAM_004");

        _time.AdvanceSeconds(40);
        var snapshot = sessions.Snapshot();

        Assert.Equal((ScanState.Idle, ScanOutcome.TimedOut), (snapshot.State, snapshot.Outcome));
        Assert.Contains("CAM_005", snapshot.LastError);
        Assert.True(sessions.Start("wf-2").Started);
    }

    [Fact]
    public void Stop_ChecksOwnershipAndCancels()
    {
        var sessions = Create();
        Assert.Equal("already_idle", sessions.Stop("人工").Code);
        sessions.Start("wf-1");

        var mismatch = sessions.Stop("人工", "wf-other");
        var stop = sessions.Stop("人工", "wf-1");

        Assert.Equal(("request_mismatch", false), (mismatch.Code, mismatch.Success));
        Assert.Equal(("cancelled", true), (stop.Code, stop.Success));
        Assert.Equal(ScanOutcome.Cancelled, stop.Snapshot.Outcome);
        Assert.Null(Frame(sessions, "CAM_004"));
    }

    [Fact]
    public void MarkPersisted_CompletesPersistenceAndFindReturnsEarlierSessions()
    {
        var sessions = Create();
        sessions.Start("wf-1");
        foreach (var camera in new[] { "CAM_004", "CAM_005", "CAM_004", "CAM_005" })
            Frame(sessions, camera, afterSeconds: 2.1);

        sessions.MarkPersisted("wf-1", "CAM_004");
        sessions.MarkPersisted("wf-1", "CAM_005");
        sessions.Start("wf-2");

        var first = sessions.Find("wf-1")!;
        Assert.Equal("saved", first.PersistenceState);
        Assert.Equal(["CAM_004", "CAM_005"], first.PersistedCameras);
        Assert.Null(sessions.Find("wf-unknown"));
        Assert.Equal("wf-2", sessions.Snapshot().RequestId);
    }
}
