import sys
sys.path.insert(0, '.')

from vector_store import vector_store

print("\n" + "="*60)
print("QDRANT DATABASE VIEWER")
print("="*60)

try:
    # Get collection info
    collection_info = vector_store.client.get_collection(vector_store.collection_name)
    
    print(f"\nCollection: {collection_info.name}")
    print(f"Total Documents: {collection_info.points_count}")
    print(f"Vector Dimensions: {collection_info.config.params.vectors.size}")
    
    # Get sample documents
    print("\n" + "="*60)
    print("SAMPLE DOCUMENTS (First 10):")
    print("="*60)
    
    results, _ = vector_store.client.scroll(
        collection_name=vector_store.collection_name,
        limit=10,
        with_payload=True,
        with_vectors=False
    )
    
    for i, point in enumerate(results, 1):
        metadata = point.payload.get('metadata', {})
        text = point.payload.get('text', '')
        
        print(f"\n[{i}] Document ID: {point.id}")
        print(f"    File: {metadata.get('file_path', 'Unknown')}")
        print(f"    Section: {metadata.get('section', 'N/A')}")
        print(f"    Text: {text[:150]}...")
        print("-" * 60)
    
    print(f"\n✅ SUCCESS! Your Qdrant database has {collection_info.points_count} documents!")
    print("="*60 + "\n")
    
except Exception as e:
    print(f"\n❌ Error: {e}")
    print("Make sure the backend has loaded the data!\n")
