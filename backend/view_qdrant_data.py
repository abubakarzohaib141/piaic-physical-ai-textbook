"""
Script to view all data stored in Qdrant Cloud
"""
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue

load_dotenv()

def view_qdrant_data():
    """View all data in Qdrant Cloud"""

    # Connect to Qdrant Cloud
    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
    )

    collection_name = "physical_ai_textbook"

    print("=" * 80)
    print("QDRANT CLOUD - VECTOR STORE DATA")
    print("=" * 80)

    # Get collection info
    try:
        collection_info = client.get_collection(collection_name)
        print(f"\nCollection Name: {collection_name}")
        print(f"Total Vectors: {collection_info.points_count}")
        print(f"Vector Size: {collection_info.config.params.vectors.size}")
        print(f"Distance Metric: {collection_info.config.params.vectors.distance}")

        # Get some sample points
        print("\n" + "=" * 80)
        print("SAMPLE TEXTBOOK CHUNKS (First 5)")
        print("=" * 80)

        # Scroll through first 10 points
        points, _ = client.scroll(
            collection_name=collection_name,
            limit=5,
            with_payload=True,
            with_vectors=False  # Don't show vectors (too large)
        )

        for i, point in enumerate(points, 1):
            print(f"\n--- Chunk {i} (ID: {point.id}) ---")
            payload = point.payload

            # Show metadata
            if 'metadata' in payload:
                metadata = payload['metadata']
                print(f"  Module: {metadata.get('module', 'N/A')}")
                print(f"  Source: {metadata.get('source', 'N/A')}")
                print(f"  Title: {metadata.get('title', 'N/A')}")
                print(f"  Chunk: {metadata.get('chunk_index', 0) + 1}/{metadata.get('total_chunks', 'N/A')}")

            # Show text preview
            if 'text' in payload:
                text = payload['text']
                # Remove emojis and special characters for display
                text_clean = text.encode('ascii', 'ignore').decode('ascii')
                preview = text_clean[:200] + "..." if len(text_clean) > 200 else text_clean
                print(f"  Text Preview: {preview}")

        # Get statistics - count all points with each module
        print("\n" + "=" * 80)
        print("STATISTICS BY MODULE")
        print("=" * 80)

        # Get all points and count by module
        all_points, _ = client.scroll(
            collection_name=collection_name,
            limit=1000,
            with_payload=True,
            with_vectors=False
        )

        module_counts = {}
        for point in all_points:
            module = point.payload.get('metadata', {}).get('module', 'Unknown')
            module_counts[module] = module_counts.get(module, 0) + 1

        for module, count in sorted(module_counts.items()):
            print(f"  {module}: {count} chunks")

        print("\n" + "=" * 80)
        print("RAG SEARCH TEST")
        print("=" * 80)

        # Test a search
        from vector_store import vector_store
        test_query = "What is ROS 2?"
        results = vector_store.search(test_query, limit=3)

        print(f"\nSearch Query: '{test_query}'")
        print(f"Results Found: {len(results)}")

        for i, result in enumerate(results, 1):
            print(f"\n  Result {i} (Score: {result['score']:.4f}):")
            print(f"    Source: {result['metadata'].get('source', 'N/A')}")
            text_clean = result['text'].encode('ascii', 'ignore').decode('ascii')
            text_preview = text_clean[:150] + "..." if len(text_clean) > 150 else text_clean
            print(f"    Text: {text_preview}")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    view_qdrant_data()
