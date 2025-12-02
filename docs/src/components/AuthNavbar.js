import React, { useState, useEffect } from 'react';
import ReactDOM from 'react-dom';
import SignupPage from './SignupPage';
import styles from './AuthNavbar.module.css';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { useAuth } from './AuthContext';

const AuthNavbar = () => {
  const [showSignup, setShowSignup] = useState(false);
  const { user, signout } = useAuth();
  const [mounted, setMounted] = useState(false);
  const [navbarContainer, setNavbarContainer] = useState(null);

  useEffect(() => {
    setMounted(true);

    // Function to find navbar
    const checkNavbar = () => {
      const nav = document.querySelector('.navbar__items--right');
      if (nav && nav !== navbarContainer) {
        setNavbarContainer(nav);
      }
    };

    // Check immediately
    checkNavbar();

    // Check periodically to handle SPA navigation
    const interval = setInterval(checkNavbar, 500);

    return () => clearInterval(interval);
  }, []);

  const handleSignupSuccess = () => {
    setShowSignup(false);
  };

  const handleLogout = () => {
    signout();
  };

  // Only run on client and when navbar is found
  if (!ExecutionEnvironment.canUseDOM || !mounted || !navbarContainer) {
    return null;
  }

  const authContent = (
    <div className={styles.authNavItem}>
      {user ? (
        <div className={styles.userInfo}>
          <span className={styles.username}>👤 {user.username}</span>
          <button onClick={handleLogout} className={styles.logoutBtn}>
            Logout
          </button>
        </div>
      ) : (
        <button onClick={() => setShowSignup(true)} className={styles.signupBtn}>
          Sign Up / Login
        </button>
      )}

      {showSignup && (
        <SignupPage
          onClose={() => setShowSignup(false)}
          onSignupSuccess={handleSignupSuccess}
        />
      )}
    </div>
  );

  return ReactDOM.createPortal(authContent, navbarContainer);
};

export default AuthNavbar;

