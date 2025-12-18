import os
import subprocess
import sys
from pathlib import Path

def install_dependencies():
    """Install required dependencies"""
    print("Installing dependencies...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"])

def start_api():
    """Start the FastAPI application"""
    print("Starting RAG Chatbot API...")
    os.chdir(os.path.join(os.path.dirname(__file__), "api"))

    # Add the parent directory to Python path so imports work correctly
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

    # Start the uvicorn server
    import uvicorn
    from main import app

    uvicorn.run(
        app,
        host=os.getenv("BACKEND_HOST", "0.0.0.0"),
        port=int(os.getenv("BACKEND_PORT", 8000)),
        reload=True  # Enable auto-reload during development
    )

if __name__ == "__main__":
    # Change to the rag_chatbot directory
    os.chdir(os.path.dirname(__file__))

    # Install dependencies if not already installed
    if "SKIP_INSTALL" not in os.environ:
        install_dependencies()

    # Start the API
    start_api()