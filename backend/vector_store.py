import os
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from typing import List, Dict
import google.generativeai as genai

class VectorStore:
    """Qdrant Cloud vector store for textbook content"""

    def __init__(self):
        # Use Qdrant Cloud - production ready!
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not qdrant_url or not qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in environment variables")

        self.client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
        )
        self.collection_name = "physical_ai_textbook"
        self.embedding_model = "models/text-embedding-004"  # Latest Gemini embedding model

        # Configure Gemini
        genai.configure(api_key=os.getenv("GOOGLE_API_KEY"))
        
    def create_collection(self, vector_size: int = 768):
        """Create Qdrant collection if it doesn't exist"""
        try:
            self.client.get_collection(self.collection_name)
            print(f"Collection '{self.collection_name}' already exists")
        except:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=vector_size,
                    distance=Distance.COSINE
                )
            )
            print(f"Created collection '{self.collection_name}'")
    
    def generate_embedding(self, text: str):
        """Generate embedding using Gemini's embedding model"""
        try:
            # Use Gemini's text-embedding-004 model
            result = genai.embed_content(
                model=self.embedding_model,
                content=text,
                task_type="retrieval_document"
            )
            return result['embedding']
        except Exception as e:
            print(f"Error generating embedding: {e}")
            raise
    
    def add_documents(self, documents: List[Dict[str, str]]):
        """
        Add documents to vector store
        documents: [{"id": "1", "text": "...", "metadata": {...}}]
        """
        points = []
        for doc in documents:
            embedding = self.generate_embedding(doc["text"])
            points.append(
                PointStruct(
                    id=int(doc["id"]),  # Convert to integer
                    vector=embedding,
                    payload={
                        "text": doc["text"],
                        "metadata": doc.get("metadata", {})
                    }
                )
            )
        
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )
        print(f"Added {len(points)} documents to vector store")
    
    def generate_query_embedding(self, query: str):
        """Generate embedding for search queries"""
        try:
            result = genai.embed_content(
                model=self.embedding_model,
                content=query,
                task_type="retrieval_query"  # Different task type for queries
            )
            return result['embedding']
        except Exception as e:
            print(f"CRITICAL ERROR generating query embedding: {e}")
            import traceback
            traceback.print_exc()
            raise

    def search(self, query: str, limit: int = 5) -> List[Dict]:
        """Search for relevant documents"""
        query_embedding = self.generate_query_embedding(query)
        
        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            limit=limit
        ).points
        
        return [
            {
                "text": hit.payload["text"],
                "metadata": hit.payload["metadata"],
                "score": hit.score
            }
            for hit in results
        ]

# Singleton instance
vector_store = VectorStore()
