from qdrant_client import QdrantClient

# Test direct connection to Qdrant
print("Testing Qdrant connection...")

try:
    client = QdrantClient(
        url="https://9590160d-84f0-4140-99b6-90ba3d723768.europe-west3-0.gcp.cloud.qdrant.io:6333",
        api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.Xg23ttZhCj5RHRdoiejSw_IoA2dwFFtc5Gm5kvQXQ6E"
    )
    
    # Try to get collections
    collections = client.get_collections()
    print(f"SUCCESS! Connected to Qdrant")
    print(f"Existing collections: {collections}")
    
except Exception as e:
    print(f"ERROR: {e}")
