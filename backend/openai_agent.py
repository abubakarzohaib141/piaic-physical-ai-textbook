"""
OpenAI Agents SDK integration with Claude via OpenAI-compatible API
Following the official pattern from Anthropic's Claude API documentation
Reference: https://ai.google.dev/gemini-api/docs/openai
"""
import os
import uuid
from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel, function_tool
from agents.run import RunConfig
from dotenv import load_dotenv
from vector_store import vector_store
from database import database

load_dotenv()

# Configure Claude via OpenAI-compatible endpoint
claude_api_key = os.getenv("GOOGLE_API_KEY")

if not claude_api_key:
    raise ValueError("GOOGLE_API_KEY is not set in environment variables")

# Create AsyncOpenAI client pointing to Gemini's OpenAI-compatible endpoint
external_client = AsyncOpenAI(
    api_key=claude_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

# Create model using Claude (via Gemini API endpoint)
model = OpenAIChatCompletionsModel(
    model="gemini-2.0-flash",
    openai_client=external_client
)

# Configure run settings
config = RunConfig(
    model=model,
    model_provider=external_client,
    tracing_disabled=True,
)

# Define RAG tool for searching textbook
@function_tool
def search_textbook(query: str) -> str:
    """
    Search the Physical AI textbook for relevant content.
    
    Args:
        query: The search query to find relevant textbook sections
        
    Returns:
        Relevant textbook content with source citations
    """
    try:
        results = vector_store.search(query, limit=3)
        if not results:
            return "No relevant content found in the textbook."
        
        context = "\n\n".join([
            f"**From {result['metadata'].get('file_path', 'unknown')}:**\n{result['text']}"
            for result in results
        ])
        return context
    except Exception as e:
        print(f"TOOL ERROR in search_textbook: {e}")
        return f"Error searching textbook: {str(e)}"

# Create the Physical AI Assistant Agent
physical_ai_agent = Agent(
    name="Physical AI Assistant",
    instructions="""You are an expert AI assistant for the Physical AI & Humanoid Robotics textbook.

Your role is to:
1. Answer questions about ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models
2. Use the search_textbook tool to find relevant content from the textbook
3. Provide accurate, detailed responses based on the textbook content
4. Explain complex robotics concepts in a clear, educational manner
5. Always cite your sources when using textbook content

When a user asks a question:
1. First use the search_textbook tool to find relevant information
2. Then provide a comprehensive answer based on that information
3. If the textbook doesn't have the answer, use your general knowledge but mention this

Be helpful, accurate, and educational!""",
    tools=[search_textbook],
    model=model,
)

async def process_query_with_openai_agents(
    message: str,
    context: str = None,
    session_id: str = None
):
    """
    Process a query using OpenAI Agents SDK with Claude backend and save to database
    """
    try:
        # Generate or use existing session ID
        if not session_id:
            session_id = str(uuid.uuid4())

        # Create session in database
        database.create_session(session_id)

        # Save user message to database
        database.add_message(session_id, "user", message, context)

        # Build enhanced prompt with context if provided (for selected text feature)
        enhanced_message = message
        if context:
            enhanced_message = f"""The user has selected this text from the book:
---
{context}
---

Their question: {message}

Please answer based on the selected text and related information from the textbook."""

        # Run the agent with the configured model
        result = await Runner.run(
            physical_ai_agent,
            input=enhanced_message,
            run_config=config
        )

        # Extract response
        response_text = result.final_output

        # Save assistant response to database
        database.add_message(session_id, "assistant", response_text)

        # Extract sources from tool calls
        sources = []
        if hasattr(result, 'messages'):
            for msg in result.messages:
                if hasattr(msg, 'tool_calls') and msg.tool_calls:
                    for tool_call in msg.tool_calls:
                        if tool_call.function.name == 'search_textbook':
                            sources.append("Physical AI Textbook")

        if not sources:
            sources = ["General Knowledge"]

        return response_text, sources, session_id
        
    except Exception as e:
        print(f"Agent error: {e}")
        # Fallback to simple Claude response
        import google.generativeai as genai
        genai.configure(api_key=claude_api_key)
        fallback_model = genai.GenerativeModel('gemini-2.0-flash')

        # Generate session ID if not already set
        if not session_id:
            session_id = str(uuid.uuid4())

        # Create session and save user message if not already done
        try:
            database.create_session(session_id)
            database.add_message(session_id, "user", message, context)
        except:
            pass  # Already saved in try block

        prompt = f"""You are a Physical AI assistant. Answer this question:

{message}

Provide a helpful response about Physical AI, ROS 2, robotics, or related topics."""

        response = fallback_model.generate_content(prompt)
        response_text = response.text

        # Save assistant response
        database.add_message(session_id, "assistant", response_text)

        return response_text, ["General Knowledge - Fallback mode"], session_id
