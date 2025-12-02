import os
import glob
from pathlib import Path
from typing import List, Dict
from dotenv import load_dotenv

# Load environment variables FIRST
load_dotenv()

from vector_store import vector_store
import hashlib

def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> List[str]:
    """Split text into overlapping chunks"""
    chunks = []
    start = 0
    text_length = len(text)
    
    while start < text_length:
        end = start + chunk_size
        chunk = text[start:end]
        chunks.append(chunk)
        start += chunk_size - overlap
    
    return chunks

def extract_metadata(file_path: str, content: str) -> Dict:
    """Extract metadata from markdown file"""
    # Get relative path from docs directory
    rel_path = str(Path(file_path).relative_to(Path(__file__).parent.parent / "docs" / "docs"))
    
    # Extract title from first heading
    lines = content.split('\n')
    title = "Unknown"
    for line in lines:
        if line.startswith('# '):
            title = line.replace('# ', '').strip()
            break
    
    # Determine module
    module = "General"
    if "module1-ros2" in rel_path:
        module = "Module 1: ROS 2"
    elif "module2-gazebo" in rel_path:
        module = "Module 2: Gazebo"
    elif "module3-isaac" in rel_path:
        module = "Module 3: NVIDIA Isaac"
    elif "module4-vla" in rel_path:
        module = "Module 4: VLA"
    
    return {
        "source": rel_path,
        "title": title,
        "module": module
    }

def ingest_textbook_content():
    """Ingest all markdown files from the textbook"""
    
    print("Starting textbook content ingestion...")
    
    # Create collection
    vector_store.create_collection()
    
    # Find all markdown files
    docs_path = Path(__file__).parent.parent / "docs" / "docs" / "physical-ai"
    md_files = glob.glob(str(docs_path / "**" / "*.md"), recursive=True)
    
    print(f"Found {len(md_files)} markdown files")
    
    all_documents = []
    doc_id = 0
    
    for file_path in md_files:
        print(f"Processing: {Path(file_path).name}")
        
        # Read file content
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Extract metadata
        metadata = extract_metadata(file_path, content)
        
        # Chunk the content
        chunks = chunk_text(content)
        
        # Create documents for each chunk
        for i, chunk in enumerate(chunks):
            all_documents.append({
                "id": str(doc_id),
                "text": chunk,
                "metadata": {
                    **metadata,
                    "chunk_index": i,
                    "total_chunks": len(chunks)
                }
            })
            doc_id += 1
    
    print(f"\nCreated {len(all_documents)} document chunks")
    print(f"Uploading to Qdrant...")
    
    # Upload in batches
    batch_size = 100
    for i in range(0, len(all_documents), batch_size):
        batch = all_documents[i:i+batch_size]
        vector_store.add_documents(batch)
        print(f"   Uploaded batch {i//batch_size + 1}/{(len(all_documents)-1)//batch_size + 1}")
    
    print(f"\nSuccessfully ingested {len(all_documents)} chunks from {len(md_files)} files!")
    print(f"Vector store is ready for RAG queries!")

if __name__ == "__main__":
    ingest_textbook_content()
