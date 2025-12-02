import os
import psycopg2
from psycopg2.extras import RealDictCursor
from datetime import datetime
from typing import List, Dict, Optional

class Database:
    """Neon Postgres database for chat history"""
    
    def __init__(self):
        self.connection_string = os.getenv("DATABASE_URL")
        self.init_tables()
    
    def get_connection(self):
        """Get database connection"""
        return psycopg2.connect(self.connection_string)
    
    def init_tables(self):
        """Initialize database tables"""
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                # Users table
                cur.execute("""
                    CREATE TABLE IF NOT EXISTS users (
                        id SERIAL PRIMARY KEY,
                        email VARCHAR(255) UNIQUE NOT NULL,
                        username VARCHAR(100) UNIQUE NOT NULL,
                        password_hash TEXT NOT NULL,
                        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                    )
                """)

                # User profiles table (for background questions)
                cur.execute("""
                    CREATE TABLE IF NOT EXISTS user_profiles (
                        id SERIAL PRIMARY KEY,
                        user_id INTEGER UNIQUE NOT NULL,
                        software_background VARCHAR(50),
                        hardware_background VARCHAR(50),
                        programming_languages TEXT,
                        robotics_experience VARCHAR(50),
                        learning_goals TEXT,
                        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                        FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
                    )
                """)

                # Chat sessions table
                cur.execute("""
                    CREATE TABLE IF NOT EXISTS chat_sessions (
                        id SERIAL PRIMARY KEY,
                        session_id VARCHAR(255) UNIQUE NOT NULL,
                        user_id INTEGER,
                        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                        FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE SET NULL
                    )
                """)

                # Chat messages table
                cur.execute("""
                    CREATE TABLE IF NOT EXISTS chat_messages (
                        id SERIAL PRIMARY KEY,
                        session_id VARCHAR(255) NOT NULL,
                        role VARCHAR(50) NOT NULL,
                        message TEXT NOT NULL,
                        context TEXT,
                        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                        FOREIGN KEY (session_id) REFERENCES chat_sessions(session_id)
                    )
                """)

                # Translation cache table
                cur.execute("""
                    CREATE TABLE IF NOT EXISTS translations (
                        id SERIAL PRIMARY KEY,
                        content_hash VARCHAR(64) UNIQUE NOT NULL,
                        original_text TEXT NOT NULL,
                        translated_text TEXT NOT NULL,
                        language VARCHAR(10) DEFAULT 'ur',
                        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                    )
                """)

                conn.commit()
    
    def create_session(self, session_id: str):
        """Create a new chat session"""
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    "INSERT INTO chat_sessions (session_id) VALUES (%s) ON CONFLICT DO NOTHING",
                    (session_id,)
                )
                conn.commit()
    
    def add_message(self, session_id: str, role: str, message: str, context: Optional[str] = None):
        """Add a message to chat history"""
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    INSERT INTO chat_messages (session_id, role, message, context)
                    VALUES (%s, %s, %s, %s)
                    """,
                    (session_id, role, message, context)
                )
                conn.commit()
    
    def get_chat_history(self, session_id: str, limit: int = 10) -> List[Dict]:
        """Get recent chat history for a session"""
        with self.get_connection() as conn:
            with conn.cursor(cursor_factory=RealDictCursor) as cur:
                cur.execute(
                    """
                    SELECT role, message, context, created_at
                    FROM chat_messages
                    WHERE session_id = %s
                    ORDER BY created_at DESC
                    LIMIT %s
                    """,
                    (session_id, limit)
                )
                results = cur.fetchall()
                return [dict(row) for row in reversed(results)]

    # Authentication methods
    def create_user(self, email: str, username: str, password_hash: str) -> Optional[int]:
        """Create a new user"""
        try:
            with self.get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        INSERT INTO users (email, username, password_hash)
                        VALUES (%s, %s, %s)
                        RETURNING id
                        """,
                        (email, username, password_hash)
                    )
                    user_id = cur.fetchone()[0]
                    conn.commit()
                    return user_id
        except psycopg2.IntegrityError:
            return None

    def get_user_by_email(self, email: str) -> Optional[Dict]:
        """Get user by email"""
        with self.get_connection() as conn:
            with conn.cursor(cursor_factory=RealDictCursor) as cur:
                cur.execute(
                    "SELECT * FROM users WHERE email = %s",
                    (email,)
                )
                result = cur.fetchone()
                return dict(result) if result else None

    def get_user_by_id(self, user_id: int) -> Optional[Dict]:
        """Get user by ID"""
        with self.get_connection() as conn:
            with conn.cursor(cursor_factory=RealDictCursor) as cur:
                cur.execute(
                    "SELECT id, email, username, created_at FROM users WHERE id = %s",
                    (user_id,)
                )
                result = cur.fetchone()
                return dict(result) if result else None

    def create_user_profile(self, user_id: int, profile_data: Dict):
        """Create user profile with background information"""
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    INSERT INTO user_profiles
                    (user_id, software_background, hardware_background,
                     programming_languages, robotics_experience, learning_goals)
                    VALUES (%s, %s, %s, %s, %s, %s)
                    ON CONFLICT (user_id) DO UPDATE SET
                        software_background = EXCLUDED.software_background,
                        hardware_background = EXCLUDED.hardware_background,
                        programming_languages = EXCLUDED.programming_languages,
                        robotics_experience = EXCLUDED.robotics_experience,
                        learning_goals = EXCLUDED.learning_goals
                    """,
                    (
                        user_id,
                        profile_data.get('software_background'),
                        profile_data.get('hardware_background'),
                        profile_data.get('programming_languages'),
                        profile_data.get('robotics_experience'),
                        profile_data.get('learning_goals')
                    )
                )
                conn.commit()

    def get_user_profile(self, user_id: int) -> Optional[Dict]:
        """Get user profile"""
        with self.get_connection() as conn:
            with conn.cursor(cursor_factory=RealDictCursor) as cur:
                cur.execute(
                    "SELECT * FROM user_profiles WHERE user_id = %s",
                    (user_id,)
                )
                result = cur.fetchone()
                return dict(result) if result else None

    # Translation cache methods
    def get_translation(self, content_hash: str) -> Optional[str]:
        """Get cached translation"""
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    "SELECT translated_text FROM translations WHERE content_hash = %s",
                    (content_hash,)
                )
                result = cur.fetchone()
                return result[0] if result else None

    def save_translation(self, content_hash: str, original_text: str, translated_text: str):
        """Save translation to cache"""
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    INSERT INTO translations (content_hash, original_text, translated_text)
                    VALUES (%s, %s, %s)
                    ON CONFLICT (content_hash) DO NOTHING
                    """,
                    (content_hash, original_text, translated_text)
                )
                conn.commit()

# Singleton instance
database = Database()
