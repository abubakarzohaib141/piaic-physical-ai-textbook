import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

print("Checking your .env configuration...\n")

# Check each variable
gemini_key = os.getenv("GOOGLE_API_KEY")
qdrant_url = os.getenv("QDRANT_URL")
qdrant_key = os.getenv("QDRANT_API_KEY")
db_url = os.getenv("DATABASE_URL")

print(f"GOOGLE_API_KEY: {'Set' if gemini_key else 'MISSING'}")
print(f"   Value: {gemini_key[:20]}..." if gemini_key else "")

print(f"\nQDRANT_URL: {'Set' if qdrant_url else 'MISSING'}")
print(f"   Value: {qdrant_url}")

print(f"\nQDRANT_API_KEY: {'Set' if qdrant_key else 'MISSING'}")
print(f"   Value: {qdrant_key[:30]}..." if qdrant_key else "")

print(f"\nDATABASE_URL: {'Set' if db_url else 'MISSING'}")
print(f"   Value: {db_url[:50]}..." if db_url else "")

print("\n" + "="*50)

# Check if Qdrant URL is correct
if qdrant_url:
    if "localhost" in qdrant_url or "127.0.0.1" in qdrant_url:
        print("ERROR: QDRANT_URL is pointing to localhost!")
        print("   It should be: https://9590160d-84f0-4140-99b6-90ba3d723768.europe-west3-0.gcp.cloud.qdrant.io:6333")
    elif "cloud.qdrant.io" in qdrant_url:
        print("OK: QDRANT_URL looks correct!")
    else:
        print("WARNING: QDRANT_URL doesn't look like a Qdrant Cloud URL")
else:
    print("QDRANT_URL is not set!")

print("\n" + "="*50)
print("\nIf any values are missing or incorrect, edit your .env file!")
