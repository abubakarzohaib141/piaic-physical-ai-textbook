import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import styles from './AuthModal.module.css';

export default function AuthModal({ isOpen, onClose, initialMode = 'signin' }) {
  const [mode, setMode] = useState(initialMode);
  const [formData, setFormData] = useState({
    email: '',
    username: '',
    password: '',
    software_background: 'beginner',
    hardware_background: 'none',
    programming_languages: '',
    robotics_experience: 'none',
    learning_goals: '',
  });
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const { signup, signin } = useAuth();

  if (!isOpen) return null;

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    let result;
    if (mode === 'signin') {
      result = await signin(formData.email, formData.password);
    } else {
      result = await signup(formData);
    }

    setLoading(false);

    if (result.success) {
      onClose();
    } else {
      setError(result.error);
    }
  };

  const handleChange = (e) => {
    setFormData({ ...formData, [e.target.name]: e.target.value });
  };

  return (
    <div className={styles.modalOverlay} onClick={onClose}>
      <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
        <button className={styles.closeButton} onClick={onClose}>
          ×
        </button>

        <h2>{mode === 'signin' ? 'Sign In' : 'Sign Up'}</h2>

        {error && <div className={styles.error}>{error}</div>}

        <form onSubmit={handleSubmit}>
          <input
            type="email"
            name="email"
            placeholder="Email"
            value={formData.email}
            onChange={handleChange}
            required
          />

          {mode === 'signup' && (
            <input
              type="text"
              name="username"
              placeholder="Username"
              value={formData.username}
              onChange={handleChange}
              required
            />
          )}

          <input
            type="password"
            name="password"
            placeholder="Password"
            value={formData.password}
            onChange={handleChange}
            required
          />

          {mode === 'signup' && (
            <>
              <h3 className={styles.sectionTitle}>Tell us about your background</h3>

              <label>Software Background:</label>
              <select
                name="software_background"
                value={formData.software_background}
                onChange={handleChange}
              >
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>

              <label>Hardware Experience:</label>
              <select
                name="hardware_background"
                value={formData.hardware_background}
                onChange={handleChange}
              >
                <option value="none">None</option>
                <option value="basic">Basic</option>
                <option value="advanced">Advanced</option>
              </select>

              <label>Programming Languages (comma-separated):</label>
              <input
                type="text"
                name="programming_languages"
                placeholder="e.g., Python, JavaScript, C++"
                value={formData.programming_languages}
                onChange={handleChange}
              />

              <label>Robotics Experience:</label>
              <select
                name="robotics_experience"
                value={formData.robotics_experience}
                onChange={handleChange}
              >
                <option value="none">None</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>

              <label>Learning Goals (optional):</label>
              <textarea
                name="learning_goals"
                placeholder="What do you want to achieve with this course?"
                value={formData.learning_goals}
                onChange={handleChange}
                rows="3"
              />
            </>
          )}

          <button type="submit" disabled={loading} className={styles.submitButton}>
            {loading ? 'Please wait...' : mode === 'signin' ? 'Sign In' : 'Sign Up'}
          </button>
        </form>

        <p className={styles.switchMode}>
          {mode === 'signin' ? (
            <>
              Don't have an account?{' '}
              <button onClick={() => setMode('signup')}>Sign Up</button>
            </>
          ) : (
            <>
              Already have an account?{' '}
              <button onClick={() => setMode('signin')}>Sign In</button>
            </>
          )}
        </p>
      </div>
    </div>
  );
}
