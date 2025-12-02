"""
Agent implementation following OpenAI Agents SDK pattern with Gemini
"""
import os
import uuid
from typing import List, Dict, Optional
import google.generativeai as genai
from dotenv import load_dotenv
from vector_store import vector_store

load_dotenv()
genai.configure(api_key=os.getenv("GOOGLE_API_KEY"))

class Tool:
    """Tool that the agent can use"""
    def __init__(self, name: str, description: str, function):
        self.name = name
        self.description = description
        self.function = function

class GeminiAgent:
    """
    Agent following OpenAI Agents SDK pattern but powered by Gemini
    """
    
    def __init__(self, name: str, instructions: str, tools: List[Tool] = None):
        self.name = name
        self.instructions = instructions
        self.tools = tools or []
        self.model = genai.GenerativeModel('gemini-2.0-flash')
        self.conversation_history = []
    
    def run(self, message: str, session_id: Optional[str] = None) -> Dict:
        """
        Run the agent with a user message (OpenAI Agents SDK pattern)
        Returns: {response: str, sources: List[str], session_id: str}
        """
        if not session_id:
            session_id = str(uuid.uuid4())
        
        # Use RAG tool to get context
        context_chunks = self._use_rag_tool(message)
        
        # Build context string
        if context_chunks:
            context_text = "\n\n".join([
                f"From {chunk['metadata'].get('file_path', 'unknown')}:\n{chunk['text']}"
                for chunk in context_chunks
            ])
            sources = [chunk['metadata'].get('file_path', 'unknown') for chunk in context_chunks]
        else:
            context_text = "No specific context found in textbook."
            sources = ["General Knowledge"]
        
        # Build the agent prompt
        agent_prompt = f"""{self.instructions}

Context from Physical AI Textbook:
{context_text}

User Question: {message}

Provide a helpful and accurate response based on the textbook content."""
        
        # Generate response using Gemini
        response = self.model.generate_content(agent_prompt)
        answer = response.text
        
        # Add to conversation history
        self.conversation_history.append({
            "role": "user",
            "content": message
        })
        self.conversation_history.append({
            "role": "assistant",
            "content": answer
        })
        
        return {
            "response": answer,
            "sources": sources,
            "session_id": session_id
        }
    
    def _use_rag_tool(self, query: str, limit: int = 3) -> List[Dict]:
        """
        RAG tool - retrieves relevant context from vector store
        """
        try:
            results = vector_store.search(query, limit=limit)
            return results
        except Exception as e:
            print(f"RAG tool error: {e}")
            return []

# Create the Physical AI Assistant Agent
physical_ai_agent = GeminiAgent(
    name="Physical AI Assistant",
    instructions="""You are an expert AI assistant for the Physical AI & Humanoid Robotics textbook.

Your role is to:
1. Answer questions about ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models
2. Use the provided textbook context to give accurate, detailed responses
3. Explain complex robotics concepts in a clear, educational manner
4. Reference specific sections of the textbook when relevant

Always prioritize accuracy and cite the textbook sources when answering.""",
    tools=[
        Tool(
            name="rag_search",
            description="Search the Physical AI textbook for relevant content",
            function=lambda query: vector_store.search(query, limit=3)
        )
    ]
)

async def process_query_with_agent(
    message: str,
    context: str = None,
    session_id: str = None
):
    """
    Process a query using the Gemini Agent (OpenAI Agents SDK pattern)
    """
    result = physical_ai_agent.run(message, session_id)
    return result["response"], result["sources"], result["session_id"]
