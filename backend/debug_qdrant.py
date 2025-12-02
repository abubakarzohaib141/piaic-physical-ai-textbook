from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

print(f"Client type: {type(client)}")
print(f"Has search: {hasattr(client, 'search')}")
print(f"Has query_points: {hasattr(client, 'query_points')}")
print(f"Dir: {dir(client)}")
