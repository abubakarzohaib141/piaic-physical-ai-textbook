from fastapi import FastAPI, HTTPException, Depends, Header
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, EmailStr
from typing import List, Optional
import os
import hashlib
from dotenv import load_dotenv
import google.generativeai as genai
import logging

# Load environment variables
load_dotenv()

print("Starting main.py...")

app = FastAPI(title="Physical AI Chatbot API")

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=os.getenv("CORS_ORIGINS", "http://localhost:3000").split(","),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Auto-load textbook content at startup
@app.on_event("startup")
async def startup_event():
    """Load textbook content into vector store on startup"""
    print("\n" + "="*50)
    print("Loading textbook content into vector store...")
    print("="*50)
    try:
        from ingestion import ingest_textbook_content
        ingest_textbook_content()
        print("\n" + "="*50)
        print("SUCCESS! Textbook content loaded!")
        print("RAG is ready to answer questions!")
        print("="*50 + "\n")
    except Exception as e:
        print(f"\nWarning: Could not load textbook content: {e}\n")

# Request/Response Models
class ChatRequest(BaseModel):
    message: str
    context: Optional[str] = None
    session_id: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    sources: List[str] = []
    session_id: str

# Auth Models
class SignupRequest(BaseModel):
    email: EmailStr
    username: str
    password: str
    software_background: str  # beginner, intermediate, advanced
    hardware_background: str  # none, basic, advanced
    programming_languages: str
    robotics_experience: str  # none, beginner, intermediate, advanced
    learning_goals: Optional[str] = None

class SigninRequest(BaseModel):
    email: EmailStr
    password: str

class AuthResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    user: dict

# Personalization Models
class PersonalizeRequest(BaseModel):
    content: str
    user_level: str  # beginner, intermediate, advanced

class PersonalizeResponse(BaseModel):
    personalized_content: str
    adjustments_made: List[str]

# Translation Models
class TranslateRequest(BaseModel):
    content: str
    target_language: str = "ur"

class TranslateResponse(BaseModel):
    translated_content: str
    cached: bool

@app.get("/")
async def root():
    """Health check endpoint"""
    return {
        "status": "online",
        "service": "Physical AI RAG Chatbot",
        "model": "Google Gemini"
    }

@app.get("/health")
async def health_check():
    """Detailed health check"""
    return {
        "status": "healthy",
        "gemini_configured": bool(os.getenv("GOOGLE_API_KEY")),
        "qdrant_cloud_configured": bool(os.getenv("QDRANT_URL") and os.getenv("QDRANT_API_KEY")),
        "neon_database_configured": bool(os.getenv("DATABASE_URL")),
        "openai_agents_sdk": True
    }

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Main chat endpoint using OpenAI Agents SDK with RAG

    Supports:
    - General questions about the textbook
    - Questions about selected text (pass the selected text in 'context' field)
    - Saves chat history to Neon Postgres
    - Uses Qdrant Cloud for vector search
    """
    try:
        from openai_agent import process_query_with_openai_agents
        response, sources, session_id = await process_query_with_openai_agents(
            message=request.message,
            context=request.context,  # Selected text from the book
            session_id=request.session_id
        )

        return ChatResponse(
            response=response,
            sources=sources,
            session_id=session_id
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/chat/history/{session_id}")
async def get_chat_history(session_id: str, limit: int = 10):
    """
    Get chat history for a session
    """
    try:
        from database import database
        history = database.get_chat_history(session_id, limit)
        return {"session_id": session_id, "history": history}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# ========== BONUS FEATURES (+150 points) ==========

# Helper function to get current user from token
async def get_current_user(authorization: Optional[str] = Header(None)):
    """Get current user from JWT token"""
    if not authorization:
        return None

    try:
        from auth_utils import verify_token
        from database import database

        token = authorization.replace("Bearer ", "")
        payload = verify_token(token)

        if not payload:
            return None

        user_id = payload.get("user_id")
        if not user_id:
            return None

        user = database.get_user_by_id(user_id)
        if user:
            user['profile'] = database.get_user_profile(user_id)

        return user
    except:
        return None

# AUTH ENDPOINTS (+50 points)

@app.post("/auth/signup", response_model=AuthResponse)
async def signup(request: SignupRequest):
    """
    User signup with background questions (+50 points bonus feature)

    Collects:
    - Software background (beginner/intermediate/advanced)
    - Hardware background (none/basic/advanced)
    - Programming languages
    - Robotics experience
    - Learning goals
    """
    try:
        from auth_utils import hash_password, create_access_token
        from database import database

        # Check if user exists
        existing_user = database.get_user_by_email(request.email)
        if existing_user:
            raise HTTPException(status_code=400, detail="Email already registered")

        # Hash password
        password_hash = hash_password(request.password)

        # Create user
        user_id = database.create_user(request.email, request.username, password_hash)
        if not user_id:
            raise HTTPException(status_code=400, detail="Username already taken")

        # Create user profile with background info
        profile_data = {
            'software_background': request.software_background,
            'hardware_background': request.hardware_background,
            'programming_languages': request.programming_languages,
            'robotics_experience': request.robotics_experience,
            'learning_goals': request.learning_goals
        }
        database.create_user_profile(user_id, profile_data)

        # Create access token
        access_token = create_access_token({"user_id": user_id, "email": request.email})

        # Get user data
        user = database.get_user_by_id(user_id)
        user['profile'] = database.get_user_profile(user_id)

        return AuthResponse(
            access_token=access_token,
            user=user
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/auth/signin", response_model=AuthResponse)
async def signin(request: SigninRequest):
    """User signin"""
    try:
        from auth_utils import verify_password, create_access_token
        from database import database

        # Get user
        user = database.get_user_by_email(request.email)
        if not user:
            raise HTTPException(status_code=401, detail="Invalid credentials")

        # Verify password
        if not verify_password(request.password, user['password_hash']):
            raise HTTPException(status_code=401, detail="Invalid credentials")

        # Create access token
        access_token = create_access_token({"user_id": user['id'], "email": user['email']})

        # Get profile
        user['profile'] = database.get_user_profile(user['id'])

        # Remove password hash from response
        del user['password_hash']

        return AuthResponse(
            access_token=access_token,
            user=user
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/auth/me")
async def get_current_user_info(current_user: dict = Depends(get_current_user)):
    """Get current logged-in user info"""
    if not current_user:
        raise HTTPException(status_code=401, detail="Not authenticated")

    return current_user

# CONTENT PERSONALIZATION (+50 points)

@app.post("/personalize", response_model=PersonalizeResponse)
async def personalize_content(request: PersonalizeRequest, current_user: dict = Depends(get_current_user)):
    """
    Personalize content based on user's background (+50 points bonus feature)

    Adjusts content difficulty based on:
    - Software background
    - Hardware experience
    - Robotics knowledge
    """
    try:
        genai.configure(api_key=os.getenv("GOOGLE_API_KEY"))
        model = genai.GenerativeModel('gemini-2.0-flash')

        # Get user level from profile or request
        user_level = request.user_level
        if current_user and current_user.get('profile'):
            profile = current_user['profile']
            # Use user's actual background
            user_level = profile.get('software_background', 'beginner')

        # Build personalization prompt
        prompt = f"""You are a content personalization assistant for a Physical AI & Robotics textbook.

User Level: {user_level}
Original Content:
{request.content}

Please personalize this content for a {user_level} level user:

For BEGINNER level:
- Add simple explanations and analogies
- Define technical terms
- Include step-by-step breakdowns
- Add encouraging notes

For INTERMEDIATE level:
- Keep technical accuracy
- Add practical examples
- Include some advanced concepts
- Link to deeper resources

For ADVANCED level:
- Focus on advanced concepts
- Include optimization tips
- Add research references
- Discuss edge cases

Return ONLY the personalized content without any meta-commentary."""

        response = model.generate_content(prompt)
        personalized = response.text

        adjustments = []
        if user_level == "beginner":
            adjustments = ["Added simple explanations", "Defined technical terms", "Added examples"]
        elif user_level == "intermediate":
            adjustments = ["Balanced technical depth", "Added practical examples"]
        else:
            adjustments = ["Focused on advanced concepts", "Added optimization tips"]

        return PersonalizeResponse(
            personalized_content=personalized,
            adjustments_made=adjustments
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# URDU TRANSLATION (+50 points)

@app.post("/translate", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    """
    Translate content to Urdu with caching (+50 points bonus feature)

    Features:
    - On-demand translation using Gemini
    - Caches translations in database
    - Fast retrieval for repeated content
    """
    try:
        from database import database

        # Create content hash for caching
        content_hash = hashlib.sha256(request.content.encode()).hexdigest()

        # Check cache first
        cached_translation = database.get_translation(content_hash)
        if cached_translation:
            return TranslateResponse(
                translated_content=cached_translation,
                cached=True
            )

        # Translate using Gemini
        genai.configure(api_key=os.getenv("GOOGLE_API_KEY"))
        model = genai.GenerativeModel('gemini-2.0-flash')

        prompt = f"""Translate the following content to Urdu.
IMPORTANT: The input is HTML/Markdown. You MUST PRESERVE all HTML tags, links, and formatting exactly.
Only translate the human-readable text content.
Keep technical terms (like ROS 2, Gazebo, Python, Node, Topic) in English.

Content to Translate:
{request.content}

Translated Content (with original HTML tags):"""

        response = model.generate_content(prompt)
        translated = response.text

        # Save to cache
        database.save_translation(content_hash, request.content, translated)

        return TranslateResponse(
            translated_content=translated,
            cached=False
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=int(os.getenv("BACKEND_PORT", 8000)), reload=False)
