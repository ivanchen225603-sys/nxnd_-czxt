namespace CarPointCloudWebApi.Domain;

public sealed record LoadRecalculationTarget(string Id, long ActivationId,
    LoadMeasurementRecord Original, LoadBaselineVersion Baseline, LoadMeasurementInput BaselineSource);

public sealed record LoadRecalculationInput(string Id, long ActivationId, string MeasurementId,
    string BaselineVersionId, LoadMeasurementInput Measurement, LoadRecalculationProvenance Provenance);

public sealed record LoadRecalculationProvenance(string InputSha256, string OriginalAlgorithmSha256,
    string CalculationAlgorithmSha256, string ExitFramesSha256);

public sealed record LoadRecalculationReceipt(string Id, string Status, LoadVolumeRevision? Revision);

public sealed record LoadVolumeRevision(string Id, string MeasurementId, long ActivationId,
    string BaselineVersionId, long CoalVolumeId, decimal? PreviousVolumeM3, decimal VolumeM3,
    string? PreviousBaselineFile, string ResultHash, LoadMeasurementInput Calculation,
    LoadRecalculationProvenance Provenance, DateTimeOffset RevisedAt);
