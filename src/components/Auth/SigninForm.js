import React, { useState } from 'react';
import './Auth.css';

const SigninForm = ({ onSwitchToSignup }) => {
  const [formData, setFormData] = useState({
    email: '',
    password: ''
  });
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    try {
      // In a real implementation, this would call an actual authentication API
      // For now, we'll simulate the process by checking if user exists in localStorage
      // In production, this should connect to a real backend authentication service

      // Simulate API call delay
      await new Promise(resolve => setTimeout(resolve, 500));

      // Check if user exists (mock implementation)
      const storedProfile = localStorage.getItem('userProfile');
      const profile = storedProfile ? JSON.parse(storedProfile) : null;

      if (profile && profile.email === formData.email) {
        // Set mock session
        localStorage.setItem('isLoggedIn', 'true');
        localStorage.setItem('authToken', `mock_token_${profile.user_id}`);

        // Successful sign in - redirect to dashboard or previous page
        window.location.href = '/dashboard';
      } else {
        setError('Invalid email or password. This is a demo - try signing up first.');
      }
    } catch (err) {
      setError(err.message || 'An error occurred during sign in');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-container">
      <div className="auth-form">
        <h2>Sign In</h2>

        {error && <div className="error-message">{error}</div>}

        <form onSubmit={handleSubmit}>
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

          <button type="submit" disabled={loading} className="btn-primary">
            {loading ? 'Signing In...' : 'Sign In'}
          </button>
        </form>

        <p className="auth-switch">
          Don't have an account?{' '}
          <button type="button" onClick={onSwitchToSignup} className="link-button">
            Sign Up
          </button>
        </p>
      </div>
    </div>
  );
};

export default SigninForm;