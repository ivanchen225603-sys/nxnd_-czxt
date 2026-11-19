using Microsoft.AspNetCore.Mvc.Testing;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public class TestFixture : IDisposable
{
    private readonly string _testUserAgent = $"CarPointCloudWebApi.Tests/{Guid.NewGuid():N}";

    public MockPartnerServer MockServer { get; }
    public TestApiFactory Factory { get; }
    public HttpClient Client { get; }
    public TestDbHelper DbHelper { get; }

    public TestFixture()
    {
        var databaseConnectionString = TestConfiguration.GetDatabaseConnectionString();
        if (databaseConnectionString is null)
        {
            // 本集合内的用例都用 [DatabaseFact]/[DatabaseTheory] 标注，没有连接串时已在
            // 发现阶段被标成 skip。但 xUnit 仍会构造 collection fixture，此处若抛异常，
            // 整个集合会变成 error 而不是 skip——所以只能安静退化。
            MockServer = null!;
            Factory = null!;
            Client = null!;
            DbHelper = null!;
            return;
        }

        MockServer = new MockPartnerServer();

        Factory = new TestApiFactory(MockServer.BaseUrl, databaseConnectionString);
        Client = Factory.CreateClient();
        Client.DefaultRequestHeaders.UserAgent.ParseAdd(_testUserAgent);
        DbHelper = new TestDbHelper(databaseConnectionString, _testUserAgent);
        DbHelper.CleanupTestData();
    }

    public void Dispose()
    {
        List<Exception>? failures = null;

        DisposeAndCapture(DbHelper, "database test-data cleanup", ref failures);
        DisposeAndCapture(Client, "HTTP client disposal", ref failures);
        DisposeAndCapture(Factory, "test server disposal", ref failures);
        DisposeAndCapture(MockServer, "mock partner server disposal", ref failures);

        if (failures is { Count: > 0 })
        {
            throw new AggregateException("Integration-test fixture cleanup failed.", failures);
        }
    }

    private static void DisposeAndCapture(
        IDisposable? disposable,
        string operation,
        ref List<Exception>? failures)
    {
        if (disposable is null)
        {
            return;
        }

        try
        {
            disposable.Dispose();
        }
        catch (Exception ex)
        {
            (failures ??= []).Add(new InvalidOperationException(
                $"Failed during {operation}.", ex));
        }
    }

}

[CollectionDefinition("Integration")]
public class IntegrationTestCollection : ICollectionFixture<TestFixture> { }
