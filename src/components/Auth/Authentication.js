import React, { useState } from 'react';
import SignupForm from './SignupForm';
import SigninForm from './SigninForm';
import './Auth.css';

const Authentication = () => {
  const [isSignup, setIsSignup] = useState(true);

  const switchToSignup = () => setIsSignup(true);
  const switchToSignin = () => setIsSignup(false);

  return (
    <div className="authentication-container">
      {isSignup ? (
        <SignupForm onSwitchToSignin={switchToSignin} />
      ) : (
        <SigninForm onSwitchToSignup={switchToSignup} />
      )}
    </div>
  );
};

export default Authentication;