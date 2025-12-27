# Authentication and Personalization System Implementation Design
## User Authentication and Personalization for Physical AI & Humanoid Robotics Book

### Overview
This document outlines the implementation plan for client-side authentication with user background collection and personalized content delivery in the Physical AI & Humanoid Robotics book project, designed for Vercel deployment compatibility.

### Requirements
- Implement signup/signin using client-side localStorage authentication
- Collect user background information during signup
- Personalize content based on user background
- Integrate with existing RAG chatbot system
- Maintain existing functionality while adding personalization
- Ensure compatibility with Vercel static site deployment

### Architecture

#### Frontend Components (Docusaurus/React)
1. **Auth Provider Component** - Wraps the application with localStorage-based authentication context
2. **Signup Form** - Multi-step form collecting user background information and storing in localStorage
3. **Signin Form** - Client-side authentication form checking localStorage
4. **User Profile Component** - Dashboard for managing preferences
5. **Protected Route Components** - Components that require authentication
6. **Enhanced RAGChatbot** - Updated component that passes user context from localStorage

### User Background Collection Schema

During signup, collect the following information:

```typescript
interface UserProfile {
  userId: string;
  name: string;
  email: string;
  createdAt: Date;

  // Background Questions
  experienceLevel: 'beginner' | 'intermediate' | 'advanced' | 'expert';
  role: 'student' | 'professional' | 'researcher' | 'hobbyist' | 'other';
  programmingExperience: string[]; // ['Python', 'C++', 'ROS', ...]
  roboticsInterest: string[]; // ['ROS 2', 'Simulation', 'Isaac', 'VLA', 'Hardware', ...]
  learningGoal: string; // 'academic', 'career', 'project', 'research', 'hobby'
  hardwareAccess: boolean; // Do they have access to physical robots?
  primaryPlatform: string; // 'Windows', 'Linux', 'MacOS', 'Other'
}
```

### Database Schema Extensions

Extend Better Auth with additional user profile information:

```sql
-- Better Auth handles user accounts
-- We extend with user profiles
CREATE TABLE user_profiles (
  id VARCHAR(255) PRIMARY KEY,
  user_id VARCHAR(255) NOT NULL,
  experience_level VARCHAR(20),
  role VARCHAR(20),
  programming_experience JSON,
  robotics_interest JSON,
  learning_goal VARCHAR(20),
  hardware_access BOOLEAN,
  primary_platform VARCHAR(10),
  preferences JSON,
  created_at TIMESTAMP,
  updated_at TIMESTAMP,
  FOREIGN KEY (user_id) REFERENCES better_auth_users(id)
);
```

### API Endpoint Modifications

#### Current RAG Endpoints (to be enhanced)
- `POST /chat` - Accept optional user_id for personalization
- `GET /documents` - Filter based on user interests
- `GET /search` - Weight results based on user profile

#### New Authentication Endpoints
- `POST /auth/signup` - Extended signup with background questions
- `GET /auth/profile` - Get user profile and preferences
- `PUT /auth/profile` - Update user preferences
- `GET /auth/personalized-content` - Get content recommendations

### Personalization Strategy

#### Content Adaptation
1. **Response Complexity**: Adjust technical depth based on experience level
   - Beginner: More explanations, step-by-step instructions
   - Advanced: Concise, technical explanations

2. **Focus Prioritization**: Emphasize content matching user interests
   - ROS 2 enthusiasts → ROS 2 content prioritized
   - Simulation-focused → Simulation content emphasized

3. **Learning Path Guidance**: Suggest appropriate next steps based on goals
   - Academic → Theory and fundamentals first
   - Career → Practical applications
   - Project-based → Hands-on examples

#### Response Customization
- Include examples relevant to user's programming experience
- Suggest hardware/software based on user's access level
- Adjust terminology based on experience level
- Provide learning path recommendations

### Frontend Implementation Plan

#### 1. Authentication Provider
Create a context provider that manages Better Auth state and user profile:

```jsx
// AuthProvider.jsx
import { useAuth } from "@better-auth/react";
import { UserProfileContext } from "./UserProfileContext";

export const AuthProvider = ({ children }) => {
  const { session, signIn, signOut } = useAuth();
  const [userProfile, setUserProfile] = useState(null);

  // Load user profile when session changes
  useEffect(() => {
    if (session?.user?.id) {
      loadUserProfile(session.user.id);
    }
  }, [session]);

  return (
    <UserProfileContext.Provider value={{
      userProfile,
      setUserProfile,
      session,
      signIn,
      signOut
    }}>
      {children}
    </UserProfileContext.Provider>
  );
};
```

#### 2. Enhanced Signup Form
Create a multi-step signup form that collects background information:

```jsx
// SignupForm.jsx
const SignupForm = () => {
  const [step, setStep] = useState(1);
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '',
    experienceLevel: '',
    role: '',
    programmingExperience: [],
    roboticsInterest: [],
    learningGoal: '',
    hardwareAccess: false,
    primaryPlatform: ''
  });

  // Multi-step form with background questions
  return (
    <div>
      {step === 1 && <BasicInfoStep {...} />}
      {step === 2 && <BackgroundStep {...} />}
      {step === 3 && <PreferencesStep {...} />}
    </div>
  );
};
```

#### 3. Updated RAGChatbot Component
Modify the existing RAGChatbot to include user context:

```jsx
// EnhancedRAGChatbot.jsx
const EnhancedRAGChatbot = ({ apiEndpoint = 'http://localhost:8000' }) => {
  const { userProfile } = useContext(UserProfileContext);

  const sendMessage = async () => {
    const payload = {
      question: inputText,
      context: selectedText || null,
      user_id: userProfile?.id, // Include user context
      user_preferences: userProfile?.preferences // Include preferences
    };

    // Send request with user context
    const response = await fetch(`${apiEndpoint}/chat`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    });
  };
};
```

### Backend Implementation Plan

#### 1. Better Auth Configuration
Set up Better Auth with custom user profile extension:

```python
# auth_config.py
from better_auth import auth, models
from pydantic import BaseModel
from typing import List, Optional

class UserProfile(BaseModel):
    experience_level: Optional[str] = None
    role: Optional[str] = None
    programming_experience: Optional[List[str]] = []
    robotics_interest: Optional[List[str]] = []
    learning_goal: Optional[str] = None
    hardware_access: Optional[bool] = False
    primary_platform: Optional[str] = None
    preferences: Optional[dict] = {}

# Initialize Better Auth with custom models
auth = auth(
    secret="your-secret-key",
    database_url="your-database-url",
    # Additional config...
)
```

#### 2. Enhanced RAG API Endpoints
Update the existing API to handle user context:

```python
# Enhanced API endpoints
@app.post("/chat")
async def chat_endpoint(request: QuestionRequest, user_id: Optional[str] = Header(None)):
    if user_id:
        # Load user profile for personalization
        user_profile = await load_user_profile(user_id)
        # Apply personalization logic
        personalized_context = await get_personalized_context(user_profile, request.question)

    # Generate response considering user context
    response = await generate_personalized_response(
        question=request.question,
        user_profile=user_profile if user_id else None
    )

    return ChatResponse(
        answer=response.answer,
        sources=response.sources,
        confidence=response.confidence
    )
```

### Security Considerations

1. **Data Protection**: User background information must be stored securely
2. **Privacy**: Provide clear privacy policy about data usage
3. **Authentication**: Use secure session management
4. **API Security**: Validate user context in all endpoints
5. **Data Minimization**: Only collect necessary information

### Migration Strategy

1. **Phase 1**: Implement basic Better Auth integration
2. **Phase 2**: Add background collection during signup
3. **Phase 3**: Implement personalization logic
4. **Phase 4**: Enhance RAG chatbot with user context
5. **Phase 5**: Add user preference management

### Testing Strategy

1. **Unit Tests**: Test authentication flows
2. **Integration Tests**: Test personalization logic
3. **E2E Tests**: Test complete signup-to-personalization flow
4. **Performance Tests**: Ensure personalization doesn't impact response times

This design provides a comprehensive approach to implementing Better Auth with user background collection and personalized content delivery while maintaining the existing functionality of the Physical AI & Humanoid Robotics book project.