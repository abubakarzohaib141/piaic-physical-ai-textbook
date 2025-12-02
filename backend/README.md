# Physical AI Textbook - RAG Chatbot Backend

Backend API for the Physical AI textbook RAG chatbot using **Google Gemini**.

## 🚀 Quick Setup

### 1. Get Your API Keys

#### **Gemini API Key** (FREE)
1. Go to https://aistudio.google.com/app/apikey
2. Click "Create API Key"
3. Copy your key

#### **Qdrant Cloud** (FREE Tier)
1. Go to https://cloud.qdrant.io/
2. Sign up and create a cluster
3. Copy your cluster URL and API key

#### **Neon Postgres** (FREE Tier)
1. Go to https://neon.tech/
2. Create a project
3. Copy your connection string

### 2. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 3. Configure Environment

```bash
# Copy the example file
cp .env.example .env

# Edit .env and add your API keys
```

Your `.env` should look like:
```
GEMINI_API_KEY=AIzaSy...
QDRANT_URL=https://xyz.qdrant.io
QDRANT_API_KEY=xyz...
DATABASE_URL=postgresql://user:pass@host/db
```

### 4. Ingest Textbook Content

```bash
python ingestion.py
```

This will:
- Read all markdown files from `docs/docs/physical-ai/`
- Split content into chunks
- Generate embeddings using Gemini
- Upload to Qdrant vector store

### 5. Start the Server

```bash
python main.py
```

Server will run on `http://localhost:8000`

## 📡 API Endpoints

### Health Check
```bash
GET http://localhost:8000/health
```

### Chat
```bash
POST http://localhost:8000/chat
Content-Type: application/json

{
  "message": "What is ROS 2?",
  "context": "optional selected text",
  "session_id": "optional-session-id"
}
```

Response:
```json
{
  "response": "ROS 2 is...",
  "sources": ["module1-ros2/overview.md"],
  "session_id": "abc-123"
}
```

## 🧪 Test the API

```bash
# Health check
curl http://localhost:8000/health

# Chat query
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Explain ROS 2 nodes"}'
```

## 📁 File Structure

```
backend/
├── main.py              # FastAPI server
├── rag_agent.py         # RAG logic with Gemini
├── vector_store.py      # Qdrant integration
├── database.py          # Neon Postgres
├── ingestion.py         # Content loader
├── requirements.txt     # Dependencies
├── .env.example         # Environment template
└── README.md           # This file
```

## 🔧 Troubleshooting

**"Module not found" error:**
```bash
pip install -r requirements.txt
```

**Database connection error:**
- Check your `DATABASE_URL` in `.env`
- Make sure Neon database is running

**Qdrant connection error:**
- Verify `QDRANT_URL` and `QDRANT_API_KEY`
- Check if cluster is active

**Gemini API error:**
- Verify `GEMINI_API_KEY` is correct
- Check API quota at https://aistudio.google.com/

## 🎯 Next Steps

1. ✅ Backend is ready!
2. 🔜 Create frontend chat UI
3. 🔜 Integrate with Docusaurus site
