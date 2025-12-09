from fastapi import FastAPI
from app.routers import query, translate, personalize

app = FastAPI(title="Physical AI & Humanoid Robotics Textbook API", version="1.0.0")

# Include routers
app.include_router(query.router, prefix="/api/query", tags=["query"])
app.include_router(translate.router, prefix="/api/translate", tags=["translate"])
app.include_router(personalize.router, prefix="/api/personalize", tags=["personalize"])

@app.get("/")
def read_root():
    return {"message": "Physical AI & Humanoid Robotics Textbook Backend API"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "textbook-backend"}