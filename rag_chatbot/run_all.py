import os
import subprocess
import threading
import time
import requests
import sys

def start_backend():
    """Start the FastAPI backend in a separate thread"""
    os.chdir(os.path.join(os.path.dirname(__file__), "api"))
    os.environ["PYTHONPATH"] = os.path.dirname(os.path.dirname(__file__))

    import uvicorn
    from main import app

    uvicorn.run(
        app,
        host=os.getenv("BACKEND_HOST", "0.0.0.0"),
        port=int(os.getenv("BACKEND_PORT", 8000)),
        log_level="info"
    )

def check_backend_health():
    """Check if the backend is running"""
    try:
        response = requests.get("http://localhost:8000/")
        return response.status_code == 200
    except:
        return False

def main():
    print("Starting Physical AI & Humanoid Robotics RAG Chatbot...")
    print("1. Starting backend API server...")

    # Start backend in a separate thread
    backend_thread = threading.Thread(target=start_backend, daemon=True)
    backend_thread.start()

    # Wait a moment for the server to start
    time.sleep(2)

    # Check if backend started successfully
    if check_backend_health():
        print("✅ Backend API server started successfully on http://localhost:8000")
        print("\n2. To start the frontend, run from the main directory:")
        print("   npm run start")
        print("\nThe RAG chatbot is now ready to use!")
        print("   - Backend: http://localhost:8000")
        print("   - Frontend: http://localhost:3000 (after starting Docusaurus)")
        print("\nPress Ctrl+C to stop the backend server.")

        try:
            # Keep the main thread alive to keep the backend running
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutting down backend server...")
            return
    else:
        print("❌ Failed to start backend server")
        return

if __name__ == "__main__":
    main()