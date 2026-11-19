using Microsoft.AspNetCore.Mvc;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class LogControllerTests
{
    [Fact]
    public void GetInteraction_ReturnsAllRepositoryHopsInOneResponse()
    {
        var repository = new FakeLogRepository
        {
            InteractionItems =
            {
                new LogEntryDto { Id = 1, Direction = "Inbound", InteractionId = "chain-001" },
                new LogEntryDto { Id = 2, Direction = "Outbound", InteractionId = "chain-001" }
            }
        };
        var controller = new LogController(repository);

        var action = controller.GetInteraction("chain-001");

        var ok = Assert.IsType<OkObjectResult>(action.Result);
        var response = Assert.IsType<ApiResponse<List<LogEntryDto>>>(ok.Value);
        Assert.Equal(200, response.Code);
        Assert.Equal(2, response.Data!.Count);
        Assert.Equal("chain-001", repository.LastInteractionId);
    }

    private sealed class FakeLogRepository : ILogRepository
    {
        public List<LogEntryDto> InteractionItems { get; } = new();
        public string? LastInteractionId { get; private set; }

        public List<LogEntryDto> QueryInteraction(string interactionId)
        {
            LastInteractionId = interactionId;
            return InteractionItems;
        }

        public List<LogEntryDto> QueryLogs(
            string? keyword,
            DateTime? startTime,
            DateTime? endTime,
            string? level,
            string? source,
            string? interactionId,
            string? workflowId,
            string? licensePlate,
            string? integrationType,
            int page,
            int pageSize,
            out long total)
        {
            total = 0;
            return new List<LogEntryDto>();
        }

        public IReadOnlyList<int> GetStatusCodesForLevel(string? level) => Array.Empty<int>();
    }
}
