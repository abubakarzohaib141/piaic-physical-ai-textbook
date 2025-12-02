import google.generativeai as genai
import os
import uuid
from typing import List, Tuple
from vector_store import vector_store
from database import database

# Configure Gemini
genai.configure(api_key=os.getenv("GOOGLE_API_KEY"))

async def process_query(
    message: str,
    context: str = None,
    session_id: str = None
) -> Tuple[str, List[str], str]:
    """
    Process user query through RAG pipeline
    
    Returns: (response, sources, session_id)
    """
    
    # Generate or use existing session ID
    if not session_id:
        session_id = str(uuid.uuid4())
        database.create_session(session_id)
    
    # Step 1: Retrieve relevant documents from vector store
    search_query = f"{context}\n\n{message}" if context else message
    relevant_docs = vector_store.search(search_query, limit=5)
    
    # Step 2: Build context from retrieved documents
    context_text = "\n\n".join([
        f"Source {i+1}:\n{doc['text']}"
        for i, doc in enumerate(relevant_docs)
    ])
    
    # Step 3: Get chat history
    chat_history = database.get_chat_history(session_id, limit=5)
    history_text = "\n".join([
        f"{msg['role']}: {msg['message']}"
        for msg in chat_history
    ])
    
    # Step 4: Build prompt for Gemini
    system_prompt = """You are an expert AI assistant for the Physical AI & Humanoid Robotics textbook.
Your role is to help students understand concepts related to ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models.

Guidelines:
- Provide clear, accurate, and educational responses
- Use the provided context from the textbook
- If you don't know something, say so
- Include code examples when relevant
- Be encouraging and supportive to learners
"""
    
    user_prompt = f"""Context from textbook:
{context_text}

Previous conversation:
{history_text}

Student's question: {message}

Please provide a helpful and accurate response based on the textbook content."""
    
    # Step 5: Generate response using Gemini
    model = genai.GenerativeModel('gemini-2.0-flash')
    response = model.generate_content(
        f"{system_prompt}\n\n{user_prompt}"
    )
    
    answer = response.text
    
    # Step 6: Save to database
    database.add_message(session_id, "user", message, context)
    database.add_message(session_id, "assistant", answer)
    
    # Step 7: Extract source references
    sources = [
        doc['metadata'].get('source', 'Unknown')
        for doc in relevant_docs[:3]
    ]
    
    return answer, sources, session_id
