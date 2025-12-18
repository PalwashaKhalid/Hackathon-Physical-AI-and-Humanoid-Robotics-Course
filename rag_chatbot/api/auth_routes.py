"""
Authentication API Routes for Physical AI & Humanoid Robotics Book
Handles user profiles and authentication integration
"""
from fastapi import APIRouter, HTTPException, Depends, Request
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import sqlite3
import json
from datetime import datetime
import os

router = APIRouter(prefix="/auth", tags=["authentication"])

# Personalization engine
class PersonalizationEngine:
    """Engine to customize content based on user profile"""

    @staticmethod
    def adjust_response_complexity(response_text: str, experience_level: str) -> str:
        """Adjust response complexity based on user experience level"""
        if not experience_level:
            return response_text

        if experience_level == 'beginner':
            # Add more explanations and context for beginners
            enhanced_response = f"**For beginners:** {response_text}\n\nThis concept might be new to you, so let me break it down:\n- Key point 1: Explanation\n- Key point 2: Explanation\n- Key point 3: Explanation\n\nFor more detailed information, check out the relevant chapter in our book."
            return enhanced_response
        elif experience_level == 'advanced' or experience_level == 'expert':
            # Provide more concise, technical explanations for advanced users
            return response_text  # Return as is for advanced users who prefer concise info
        else:
            return response_text

    @staticmethod
    def prioritize_content_chunks(chunks: List[Dict], user_profile: Dict) -> List[Dict]:
        """Re-rank content chunks based on user interests and preferences"""
        if not user_profile:
            return chunks

        # Get user interests
        interests = user_profile.get('robotics_interest', [])
        experience_level = user_profile.get('experience_level')

        # Score each chunk based on relevance to user interests
        scored_chunks = []
        for chunk in chunks:
            score = 0

            # Boost score if chunk contains user interests
            content_lower = chunk['content'].lower()
            for interest in interests:
                if interest.replace('_', ' ') in content_lower:
                    score += 1

            # Adjust score based on experience level
            if experience_level == 'beginner':
                # Prefer chunks that are more explanatory
                if 'introduction' in content_lower or 'overview' in content_lower or 'basics' in content_lower:
                    score += 0.5
            elif experience_level in ['advanced', 'expert']:
                # Prefer chunks with more technical depth
                if 'implementation' in content_lower or 'advanced' in content_lower or 'optimization' in content_lower:
                    score += 0.5

            scored_chunks.append((chunk, score))

        # Sort by score (highest first) and return chunks
        scored_chunks.sort(key=lambda x: x[1], reverse=True)
        return [chunk for chunk, score in scored_chunks]

    @staticmethod
    def get_personalized_recommendations(user_profile: Dict) -> List[Dict]:
        """Generate personalized content recommendations based on user profile"""
        recommendations = []

        if not user_profile:
            return recommendations

        # Based on experience level
        if user_profile.get('experience_level') == 'beginner':
            recommendations.append({
                "type": "content",
                "title": "Start with Fundamentals",
                "description": "We recommend beginning with the fundamentals chapter",
                "link": "/docs/chapter1-fundamentals",
                "priority": "high"
            })
        elif user_profile.get('experience_level') == 'advanced':
            recommendations.append({
                "type": "content",
                "title": "Advanced Topics",
                "description": "Based on your experience, you might enjoy advanced topics",
                "link": "/docs/chapter7-advanced-topics",
                "priority": "high"
            })

        # Based on robotics interests
        interests = user_profile.get('robotics_interest', [])
        if 'ros2' in interests:
            recommendations.append({
                "type": "content",
                "title": "ROS 2 Resources",
                "description": "Content related to ROS 2 based on your interests",
                "link": "/docs/chapter2-ros2",
                "priority": "medium"
            })

        if 'simulation' in interests:
            recommendations.append({
                "type": "content",
                "title": "Simulation Resources",
                "description": "Content related to simulation based on your interests",
                "link": "/docs/chapter3-simulation",
                "priority": "medium"
            })

        if 'isaac' in interests:
            recommendations.append({
                "type": "content",
                "title": "NVIDIA Isaac Resources",
                "description": "Content related to NVIDIA Isaac based on your interests",
                "link": "/docs/chapter4-isaac",
                "priority": "medium"
            })

        # Based on learning goal
        learning_goal = user_profile.get('learning_goal')
        if learning_goal == 'project':
            recommendations.append({
                "type": "content",
                "title": "Project-Based Learning",
                "description": "Practical content for your project-based learning goal",
                "link": "/docs/chapter6-complete-system",
                "priority": "high"
            })
        elif learning_goal == 'academic':
            recommendations.append({
                "type": "content",
                "title": "Academic Content",
                "description": "Theory-focused content for academic learning",
                "link": "/docs/chapter1-fundamentals",
                "priority": "high"
            })

        return recommendations

# User profile model
class UserProfile(BaseModel):
    user_id: str
    experience_level: Optional[str] = None
    role: Optional[str] = None
    programming_experience: Optional[List[str]] = []
    robotics_interest: Optional[List[str]] = []
    learning_goal: Optional[str] = None
    hardware_access: Optional[bool] = False
    primary_platform: Optional[str] = None
    preferences: Optional[Dict[str, Any]] = {}

class UserProfileUpdate(BaseModel):
    experience_level: Optional[str] = None
    role: Optional[str] = None
    programming_experience: Optional[List[str]] = None
    robotics_interest: Optional[List[str]] = None
    learning_goal: Optional[str] = None
    hardware_access: Optional[bool] = None
    primary_platform: Optional[str] = None
    preferences: Optional[Dict[str, Any]] = None

# Database setup
def init_db():
    """Initialize the user profiles database"""
    conn = sqlite3.connect('user_profiles.db')
    cursor = conn.cursor()

    # Create user_profiles table
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS user_profiles (
            user_id TEXT PRIMARY KEY,
            experience_level TEXT,
            role TEXT,
            programming_experience TEXT,
            robotics_interest TEXT,
            learning_goal TEXT,
            hardware_access BOOLEAN DEFAULT 0,
            primary_platform TEXT,
            preferences TEXT,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    ''')

    conn.commit()
    conn.close()

def get_db_connection():
    """Get database connection"""
    return sqlite3.connect('user_profiles.db')

# Initialize database
init_db()

@router.post("/profile")
async def create_user_profile(profile: UserProfile):
    """Create or update user profile with background information"""
    try:
        conn = get_db_connection()
        cursor = conn.cursor()

        # Convert lists to JSON strings for storage
        cursor.execute('''
            INSERT OR REPLACE INTO user_profiles
            (user_id, experience_level, role, programming_experience, robotics_interest,
             learning_goal, hardware_access, primary_platform, preferences, created_at, updated_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?,
                    COALESCE((SELECT created_at FROM user_profiles WHERE user_id = ?), CURRENT_TIMESTAMP),
                    CURRENT_TIMESTAMP)
        ''', (
            profile.user_id,
            profile.experience_level,
            profile.role,
            json.dumps(profile.programming_experience),
            json.dumps(profile.robotics_interest),
            profile.learning_goal,
            profile.hardware_access,
            profile.primary_platform,
            json.dumps(profile.preferences),
            profile.user_id  # for the created_at fallback
        ))

        conn.commit()
        conn.close()

        return {"message": "Profile created/updated successfully", "user_id": profile.user_id}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating profile: {str(e)}")

@router.get("/profile/{user_id}")
async def get_user_profile(user_id: str):
    """Get user profile by user ID"""
    try:
        conn = get_db_connection()
        cursor = conn.cursor()

        cursor.execute('''
            SELECT user_id, experience_level, role, programming_experience, robotics_interest,
                   learning_goal, hardware_access, primary_platform, preferences, created_at, updated_at
            FROM user_profiles WHERE user_id = ?
        ''', (user_id,))

        row = cursor.fetchone()
        conn.close()

        if not row:
            raise HTTPException(status_code=404, detail="User profile not found")

        # Convert JSON strings back to Python objects
        return {
            "user_id": row[0],
            "experience_level": row[1],
            "role": row[2],
            "programming_experience": json.loads(row[3]) if row[3] else [],
            "robotics_interest": json.loads(row[4]) if row[4] else [],
            "learning_goal": row[5],
            "hardware_access": bool(row[6]),
            "primary_platform": row[7],
            "preferences": json.loads(row[8]) if row[8] else {},
            "created_at": row[9],
            "updated_at": row[10]
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error fetching profile: {str(e)}")

@router.put("/profile/{user_id}")
async def update_user_profile(user_id: str, profile_update: UserProfileUpdate):
    """Update user profile"""
    try:
        conn = get_db_connection()
        cursor = conn.cursor()

        # Get current profile to merge updates
        cursor.execute('SELECT * FROM user_profiles WHERE user_id = ?', (user_id,))
        current_profile = cursor.fetchone()

        if not current_profile:
            raise HTTPException(status_code=404, detail="User profile not found")

        # Prepare update values, using existing values where no update is provided
        experience_level = profile_update.experience_level or current_profile[1]
        role = profile_update.role or current_profile[2]
        programming_experience = profile_update.programming_experience or json.loads(current_profile[3]) if current_profile[3] else []
        robotics_interest = profile_update.robotics_interest or json.loads(current_profile[4]) if current_profile[4] else []
        learning_goal = profile_update.learning_goal or current_profile[5]
        hardware_access = profile_update.hardware_access if profile_update.hardware_access is not None else bool(current_profile[6])
        primary_platform = profile_update.primary_platform or current_profile[7]
        preferences = profile_update.preferences or json.loads(current_profile[8]) if current_profile[8] else {}

        cursor.execute('''
            UPDATE user_profiles
            SET experience_level = ?, role = ?, programming_experience = ?, robotics_interest = ?,
                learning_goal = ?, hardware_access = ?, primary_platform = ?, preferences = ?, updated_at = CURRENT_TIMESTAMP
            WHERE user_id = ?
        ''', (
            experience_level,
            role,
            json.dumps(programming_experience),
            json.dumps(robotics_interest),
            learning_goal,
            hardware_access,
            primary_platform,
            json.dumps(preferences),
            user_id
        ))

        conn.commit()
        conn.close()

        return {"message": "Profile updated successfully", "user_id": user_id}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating profile: {str(e)}")

@router.get("/personalized-content/{user_id}")
async def get_personalized_content(user_id: str):
    """Get personalized content recommendations based on user profile"""
    try:
        profile = await get_user_profile(user_id)

        # Generate personalized content recommendations based on user profile
        recommendations = []

        # Based on experience level
        if profile.get('experience_level') == 'beginner':
            recommendations.append({
                "type": "content",
                "title": "Start with Fundamentals",
                "description": "We recommend beginning with the fundamentals chapter",
                "link": "/docs/chapter1-fundamentals",
                "priority": "high"
            })
        elif profile.get('experience_level') == 'advanced':
            recommendations.append({
                "type": "content",
                "title": "Advanced Topics",
                "description": "Based on your experience, you might enjoy advanced topics",
                "link": "/docs/chapter7-advanced-topics",
                "priority": "high"
            })

        # Based on robotics interests
        interests = profile.get('robotics_interest', [])
        if 'ros2' in interests:
            recommendations.append({
                "type": "content",
                "title": "ROS 2 Resources",
                "description": "Content related to ROS 2 based on your interests",
                "link": "/docs/chapter2-ros2",
                "priority": "medium"
            })

        if 'simulation' in interests:
            recommendations.append({
                "type": "content",
                "title": "Simulation Resources",
                "description": "Content related to simulation based on your interests",
                "link": "/docs/chapter3-simulation",
                "priority": "medium"
            })

        if 'isaac' in interests:
            recommendations.append({
                "type": "content",
                "title": "NVIDIA Isaac Resources",
                "description": "Content related to NVIDIA Isaac based on your interests",
                "link": "/docs/chapter4-isaac",
                "priority": "medium"
            })

        # Based on learning goal
        learning_goal = profile.get('learning_goal')
        if learning_goal == 'project':
            recommendations.append({
                "type": "content",
                "title": "Project-Based Learning",
                "description": "Practical content for your project-based learning goal",
                "link": "/docs/chapter6-complete-system",
                "priority": "high"
            })
        elif learning_goal == 'academic':
            recommendations.append({
                "type": "content",
                "title": "Academic Content",
                "description": "Theory-focused content for academic learning",
                "link": "/docs/chapter1-fundamentals",
                "priority": "high"
            })

        return {
            "user_id": user_id,
            "recommendations": recommendations,
            "profile_summary": {
                "experience_level": profile.get('experience_level'),
                "interests": interests,
                "learning_goal": profile.get('learning_goal')
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting personalized content: {str(e)}")

# Add this router to your main FastAPI app