import requests
import json

print("Testing backend API...")

# Test health endpoint
try:
    response = requests.get("http://localhost:8000/health")
    print(f"\nHealth Check: {response.json()}")
except Exception as e:
    print(f"Health check failed: {e}")

# Test chat endpoint
try:
    response = requests.post(
        "http://localhost:8000/chat",
        json={"message": "What is ROS 2?"}
    )
    if response.status_code == 200:
        result = response.json()
        print(f"\nChat Response:")
        print(f"Answer: {result['response'][:200]}...")
        print(f"Sources: {result['sources']}")
        print(f"Session ID: {result['session_id']}")
        print("\nSUCCESS! Backend is working!")
    else:
        print(f"\nChat failed with status {response.status_code}")
        print(f"Error: {response.text}")
except Exception as e:
    print(f"Chat failed: {e}")
    import traceback
    traceback.print_exc()
