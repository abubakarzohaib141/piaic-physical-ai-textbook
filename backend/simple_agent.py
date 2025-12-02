"""
Simple chatbot without RAG - just uses Gemini directly
This is a fallback if Qdrant connection fails
"""
import google.generativeai as genai
import os
from dotenv import load_dotenv
from typing import Tuple, List

load_dotenv()

# Configure Gemini
genai.configure(api_key=os.getenv("GOOGLE_API_KEY"))

async def process_query_simple(
    message: str,
    context: str = None,
    session_id: str = None
) -> Tuple[str, List[str], str]:
    """
    Simple query processing without RAG
    Just uses Gemini with a system prompt about Physical AI
    """
    
    # Generate session ID if needed
    if not session_id:
        import uuid
        session_id = str(uuid.uuid4())
    
    # Build prompt
    system_prompt = """You are an expert AI assistant for the Physical AI & Humanoid Robotics textbook.
You help students learn about:
- ROS 2 (Robot Operating System)
- Gazebo and Unity simulation
- NVIDIA Isaac platform
- Vision-Language-Action (VLA) models
- Humanoid robotics

Provide clear, educational responses. Include code examples when relevant.
Be encouraging and supportive to learners."""
    
    if context:
        user_prompt = f"""Context from textbook: {context}

Student's question: {message}

Please provide a helpful response based on Physical AI and robotics concepts."""
    else:
        user_prompt = f"""Student's question: {message}

Please provide a helpful response about Physical AI, ROS 2, robotics simulation, or VLA models."""
    
    # Generate response
    model = genai.GenerativeModel('gemini-2.0-flash')
    response = model.generate_content(f"{system_prompt}\n\n{user_prompt}")
    
    answer = response.text
    sources = ["General Knowledge - RAG disabled"]
    
    return answer, sources, session_id
