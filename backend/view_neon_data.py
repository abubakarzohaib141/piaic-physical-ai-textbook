"""
Script to view all data stored in Neon Postgres database
"""
import os
from dotenv import load_dotenv
import psycopg2
from psycopg2.extras import RealDictCursor

load_dotenv()

def view_neon_data():
    """View all data in Neon database"""
    conn = psycopg2.connect(os.getenv("DATABASE_URL"))

    print("=" * 80)
    print("NEON POSTGRES DATABASE - CHAT SESSIONS")
    print("=" * 80)

    # View chat sessions
    with conn.cursor(cursor_factory=RealDictCursor) as cur:
        cur.execute("SELECT * FROM chat_sessions ORDER BY created_at DESC")
        sessions = cur.fetchall()
        print(f"\nTotal Sessions: {len(sessions)}")
        print("\nRecent Sessions:")
        for session in sessions[:5]:  # Show latest 5
            print(f"  - Session ID: {session['session_id']}")
            print(f"    Created: {session['created_at']}")

    print("\n" + "=" * 80)
    print("NEON POSTGRES DATABASE - CHAT MESSAGES")
    print("=" * 80)

    # View chat messages
    with conn.cursor(cursor_factory=RealDictCursor) as cur:
        cur.execute("SELECT COUNT(*) as count FROM chat_messages")
        count = cur.fetchone()['count']
        print(f"\nTotal Messages: {count}")

        cur.execute("""
            SELECT session_id, role,
                   LEFT(message, 100) as message_preview,
                   LEFT(context, 50) as context_preview,
                   created_at
            FROM chat_messages
            ORDER BY created_at DESC
            LIMIT 10
        """)
        messages = cur.fetchall()

        print("\nRecent Messages:")
        for msg in messages:
            print(f"\n  Session: {msg['session_id']}")
            print(f"  Role: {msg['role']}")
            print(f"  Message: {msg['message_preview']}...")
            if msg['context_preview']:
                print(f"  Context: {msg['context_preview']}...")
            print(f"  Time: {msg['created_at']}")

    conn.close()

if __name__ == "__main__":
    view_neon_data()
