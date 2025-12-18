import React, { useState } from 'react';
import { useAuth } from '@better-auth/react';
import './Auth.css';

const SignupForm = ({ onSwitchToSignin }) => {
  const { signUp } = useAuth();
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
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleInputChange = (e) => {
    const { name, value, type, checked } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: type === 'checkbox' ? checked : value
    }));
  };

  const handleArrayChange = (e, field) => {
    const { value, checked } = e.target;
    setFormData(prev => ({
      ...prev,
      [field]: checked
        ? [...prev[field], value]
        : prev[field].filter(item => item !== value)
    }));
  };

  const handleNext = () => {
    setStep(prev => prev + 1);
  };

  const handleBack = () => {
    setStep(prev => prev - 1);
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    try {
      // First, create the basic account
      const result = await signUp({
        email: formData.email,
        password: formData.password,
        name: formData.name
      });

      if (result?.error) {
        setError(result.error.message);
        setLoading(false);
        return;
      }

      // Then, save the profile information
      const profileResponse = await fetch('/api/auth/profile', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          userId: result.userId || result.user.id, // depending on response format
          ...formData
        })
      });

      if (!profileResponse.ok) {
        setError('Failed to save profile information');
        setLoading(false);
        return;
      }

      // Success - user can now be redirected
      window.location.href = '/dashboard'; // or wherever you want to redirect
    } catch (err) {
      setError(err.message || 'An error occurred during signup');
    } finally {
      setLoading(false);
    }
  };

  const experienceLevels = [
    { value: 'beginner', label: 'Beginner (New to robotics)' },
    { value: 'intermediate', label: 'Intermediate (Some experience)' },
    { value: 'advanced', label: 'Advanced (Significant experience)' },
    { value: 'expert', label: 'Expert (Professional/researcher)' }
  ];

  const roles = [
    { value: 'student', label: 'Student' },
    { value: 'professional', label: 'Professional' },
    { value: 'researcher', label: 'Researcher' },
    { value: 'hobbyist', label: 'Hobbyist' },
    { value: 'other', label: 'Other' }
  ];

  const programmingSkills = [
    { value: 'python', label: 'Python' },
    { value: 'cpp', label: 'C++' },
    { value: 'ros', label: 'ROS/ROS 2' },
    { value: 'javascript', label: 'JavaScript' },
    { value: 'typescript', label: 'TypeScript' },
    { value: 'java', label: 'Java' },
    { value: 'csharp', label: 'C#' }
  ];

  const roboticsAreas = [
    { value: 'ros2', label: 'ROS 2' },
    { value: 'simulation', label: 'Simulation' },
    { value: 'isaac', label: 'NVIDIA Isaac' },
    { value: 'vla', label: 'Vision-Language-Action Systems' },
    { value: 'control', label: 'Control Systems' },
    { value: 'computer_vision', label: 'Computer Vision' },
    { value: 'machine_learning', label: 'Machine Learning' },
    { value: 'hardware', label: 'Hardware' }
  ];

  const learningGoals = [
    { value: 'academic', label: 'Academic Learning' },
    { value: 'career', label: 'Career Development' },
    { value: 'project', label: 'Personal Project' },
    { value: 'research', label: 'Research' },
    { value: 'hobby', label: 'Hobby/Interest' }
  ];

  const platforms = [
    { value: 'windows', label: 'Windows' },
    { value: 'linux', label: 'Linux' },
    { value: 'macos', label: 'macOS' },
    { value: 'other', label: 'Other' }
  ];

  return (
    <div className="auth-container">
      <div className="auth-form">
        <h2>Sign Up</h2>

        {error && <div className="error-message">{error}</div>}

        <form onSubmit={handleSubmit}>
          {/* Step 1: Basic Information */}
          {step === 1 && (
            <div className="auth-step">
              <h3>Basic Information</h3>
              <div className="form-group">
                <label htmlFor="name">Full Name</label>
                <input
                  type="text"
                  id="name"
                  name="name"
                  value={formData.name}
                  onChange={handleInputChange}
                  required
                />
              </div>

              <div className="form-group">
                <label htmlFor="email">Email</label>
                <input
                  type="email"
                  id="email"
                  name="email"
                  value={formData.email}
                  onChange={handleInputChange}
                  required
                />
              </div>

              <div className="form-group">
                <label htmlFor="password">Password</label>
                <input
                  type="password"
                  id="password"
                  name="password"
                  value={formData.password}
                  onChange={handleInputChange}
                  required
                />
              </div>
            </div>
          )}

          {/* Step 2: Experience Level */}
          {step === 2 && (
            <div className="auth-step">
              <h3>Your Background</h3>

              <div className="form-group">
                <label>Experience Level in Robotics</label>
                {experienceLevels.map(level => (
                  <div key={level.value} className="radio-option">
                    <input
                      type="radio"
                      id={`exp-${level.value}`}
                      name="experienceLevel"
                      value={level.value}
                      checked={formData.experienceLevel === level.value}
                      onChange={handleInputChange}
                      required
                    />
                    <label htmlFor={`exp-${level.value}`}>{level.label}</label>
                  </div>
                ))}
              </div>

              <div className="form-group">
                <label>Role/Identity</label>
                {roles.map(role => (
                  <div key={role.value} className="radio-option">
                    <input
                      type="radio"
                      id={`role-${role.value}`}
                      name="role"
                      value={role.value}
                      checked={formData.role === role.value}
                      onChange={handleInputChange}
                      required
                    />
                    <label htmlFor={`role-${role.value}`}>{role.label}</label>
                  </div>
                ))}
              </div>
            </div>
          )}

          {/* Step 3: Technical Skills */}
          {step === 3 && (
            <div className="auth-step">
              <h3>Technical Background</h3>

              <div className="form-group">
                <label>Programming Experience (select all that apply)</label>
                {programmingSkills.map(skill => (
                  <div key={skill.value} className="checkbox-option">
                    <input
                      type="checkbox"
                      id={`skill-${skill.value}`}
                      value={skill.value}
                      checked={formData.programmingExperience.includes(skill.value)}
                      onChange={(e) => handleArrayChange(e, 'programmingExperience')}
                    />
                    <label htmlFor={`skill-${skill.value}`}>{skill.label}</label>
                  </div>
                ))}
              </div>

              <div className="form-group">
                <label>Robotics Areas of Interest (select all that apply)</label>
                {roboticsAreas.map(area => (
                  <div key={area.value} className="checkbox-option">
                    <input
                      type="checkbox"
                      id={`area-${area.value}`}
                      value={area.value}
                      checked={formData.roboticsInterest.includes(area.value)}
                      onChange={(e) => handleArrayChange(e, 'roboticsInterest')}
                    />
                    <label htmlFor={`area-${area.value}`}>{area.label}</label>
                  </div>
                ))}
              </div>
            </div>
          )}

          {/* Step 4: Goals and Preferences */}
          {step === 4 && (
            <div className="auth-step">
              <h3>Your Goals</h3>

              <div className="form-group">
                <label>Learning Goal</label>
                {learningGoals.map(goal => (
                  <div key={goal.value} className="radio-option">
                    <input
                      type="radio"
                      id={`goal-${goal.value}`}
                      name="learningGoal"
                      value={goal.value}
                      checked={formData.learningGoal === goal.value}
                      onChange={handleInputChange}
                      required
                    />
                    <label htmlFor={`goal-${goal.value}`}>{goal.label}</label>
                  </div>
                ))}
              </div>

              <div className="form-group">
                <label htmlFor="primaryPlatform">Primary Development Platform</label>
                <select
                  id="primaryPlatform"
                  name="primaryPlatform"
                  value={formData.primaryPlatform}
                  onChange={handleInputChange}
                  required
                >
                  <option value="">Select a platform</option>
                  {platforms.map(platform => (
                    <option key={platform.value} value={platform.value}>
                      {platform.label}
                    </option>
                  ))}
                </select>
              </div>

              <div className="form-group">
                <label>
                  <input
                    type="checkbox"
                    name="hardwareAccess"
                    checked={formData.hardwareAccess}
                    onChange={handleInputChange}
                  />
                  I have access to physical robotics hardware
                </label>
              </div>
            </div>
          )}

          {/* Navigation buttons */}
          <div className="form-navigation">
            {step > 1 && (
              <button type="button" onClick={handleBack} className="btn-secondary">
                Back
              </button>
            )}

            {step < 4 ? (
              <button type="button" onClick={handleNext} className="btn-primary">
                Next
              </button>
            ) : (
              <button type="submit" disabled={loading} className="btn-primary">
                {loading ? 'Creating Account...' : 'Create Account'}
              </button>
            )}
          </div>
        </form>

        <p className="auth-switch">
          Already have an account?{' '}
          <button type="button" onClick={onSwitchToSignin} className="link-button">
            Sign In
          </button>
        </p>
      </div>
    </div>
  );
};

export default SignupForm;