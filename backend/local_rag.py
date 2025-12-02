"""
Simple local RAG without Qdrant - reads markdown files directly
"""
import os
import glob
from pathlib import Path
import google.generativeai as genai
from dotenv import load_dotenv

load_dotenv()
genai.configure(api_key=os.getenv("GOOGLE_API_KEY"))

def load_textbook_content():
    """Load all markdown files from the textbook"""
    docs_path = Path(__file__).parent.parent / "docs" / "docs" / "physical-ai"
    md_files = glob.glob(str(docs_path / "**" / "*.md"), recursive=True)
    
    content_dict = {}
    for file_path in md_files:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            # Get relative path as key
            rel_path = str(Path(file_path).relative_to(docs_path))
            content_dict[rel_path] = content
    
    return content_dict

# Load content once at startup
TEXTBOOK_CONTENT = load_textbook_content()

async def process_query_local_rag(
    message: str,
    context: str = None,
    session_id: str = None
):
    """
    Simple local RAG - searches markdown files for relevant content
    """
    import uuid
    if not session_id:
        session_id = str(uuid.uuid4())
    
    # Simple keyword search in content
    relevant_chunks = []
    search_terms = message.lower().split()
    
    for file_path, content in TEXTBOOK_CONTENT.items():
        content_lower = content.lower()
        # Count how many search terms appear
        matches = sum(1 for term in search_terms if term in content_lower)
        if matches > 0:
            # Get first 1000 chars as a chunk
            relevant_chunks.append({
                'source': file_path,
                'content': content[:1000],
                'score': matches
            })
    
    # Sort by relevance and take top 3
    relevant_chunks.sort(key=lambda x: x['score'], reverse=True)
    top_chunks = relevant_chunks[:3]
    
    # Build context
    context_text = "\n\n".join([
        f"From {chunk['source']}:\n{chunk['content']}"
        for chunk in top_chunks
    ])
    
    # Build prompt
    system_prompt = """You are an expert AI assistant for the Physical AI & Humanoid Robotics textbook.
Use the provided context from the textbook to answer questions accurately.
If the context doesn't contain the answer, use your knowledge but mention it's general knowledge."""
    
    user_prompt = f"""Context from textbook:
{context_text}

Student's question: {message}

Please provide a helpful and accurate response based on the textbook content."""
    
    # Generate response
    model = genai.GenerativeModel('gemini-2.0-flash')
    response = model.generate_content(f"{system_prompt}\n\n{user_prompt}")
    
    answer = response.text
    sources = [chunk['source'] for chunk in top_chunks] if top_chunks else ["No specific source found"]
    
    return answer, sources, session_id
