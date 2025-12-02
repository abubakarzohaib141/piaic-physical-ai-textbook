import React, { useState } from 'react';
import ReactDOM from 'react-dom';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import styles from './SignupPage.module.css';

import { useAuth } from './AuthContext';

const SignupPage = ({ onClose, onSignupSuccess }) => {
    const { signup } = useAuth();
    const [formData, setFormData] = useState({
        email: '',
        username: '',
        password: '',
        confirmPassword: '',
        software_background: 'beginner',
        hardware_background: 'none',
        programming_languages: '',
        robotics_experience: 'none',
        learning_goals: ''
    });
    const [error, setError] = useState('');
    const [loading, setLoading] = useState(false);

    const handleChange = (e) => {
        setFormData({
            ...formData,
            [e.target.name]: e.target.value
        });
    };

    const handleSubmit = async (e) => {
        e.preventDefault();
        setError('');

        // Validation
        if (formData.password !== formData.confirmPassword) {
            setError('Passwords do not match');
            return;
        }

        if (formData.password.length < 6) {
            setError('Password must be at least 6 characters');
            return;
        }

        setLoading(true);

        try {
            const result = await signup({
                email: formData.email,
                username: formData.username,
                password: formData.password,
                software_background: formData.software_background,
                hardware_background: formData.hardware_background,
                programming_languages: formData.programming_languages,
                robotics_experience: formData.robotics_experience,
                learning_goals: formData.learning_goals
            });

            if (result.success) {
                // Call success callback
                if (onSignupSuccess) {
                    onSignupSuccess();
                }

                // Close modal
                if (onClose) {
                    onClose();
                }
            } else {
                setError(result.error || 'Signup failed');
            }
        } catch (err) {
            setError('An unexpected error occurred');
        } finally {
            setLoading(false);
        }
    };

    if (!ExecutionEnvironment.canUseDOM) {
        return null;
    }

    return ReactDOM.createPortal(
        <div className={styles.overlay}>
            <div className={styles.modal}>
                <button className={styles.closeBtn} onClick={onClose}>×</button>

                <h2 className={styles.title}>Join Physical AI Learning</h2>
                <p className={styles.subtitle}>Create your account to get personalized content</p>

                {error && <div className={styles.error}>{error}</div>}

                <form onSubmit={handleSubmit} className={styles.form}>
                    <div className={styles.formGroup}>
                        <label>Email</label>
                        <input
                            type="email"
                            name="email"
                            value={formData.email}
                            onChange={handleChange}
                            required
                            placeholder="your@email.com"
                        />
                    </div>

                    <div className={styles.formGroup}>
                        <label>Username</label>
                        <input
                            type="text"
                            name="username"
                            value={formData.username}
                            onChange={handleChange}
                            required
                            placeholder="Choose a username"
                        />
                    </div>

                    <div className={styles.formRow}>
                        <div className={styles.formGroup}>
                            <label>Password</label>
                            <input
                                type="password"
                                name="password"
                                value={formData.password}
                                onChange={handleChange}
                                required
                                placeholder="Min 6 characters"
                            />
                        </div>

                        <div className={styles.formGroup}>
                            <label>Confirm Password</label>
                            <input
                                type="password"
                                name="confirmPassword"
                                value={formData.confirmPassword}
                                onChange={handleChange}
                                required
                                placeholder="Confirm password"
                            />
                        </div>
                    </div>

                    <div className={styles.section}>
                        <h3>Tell us about yourself</h3>
                        <p className={styles.sectionDesc}>This helps us personalize your learning experience</p>
                    </div>

                    <div className={styles.formGroup}>
                        <label>Software Background</label>
                        <select
                            name="software_background"
                            value={formData.software_background}
                            onChange={handleChange}
                        >
                            <option value="beginner">Beginner - New to programming</option>
                            <option value="intermediate">Intermediate - Some experience</option>
                            <option value="advanced">Advanced - Professional developer</option>
                        </select>
                    </div>

                    <div className={styles.formGroup}>
                        <label>Hardware Experience</label>
                        <select
                            name="hardware_background"
                            value={formData.hardware_background}
                            onChange={handleChange}
                        >
                            <option value="none">None - Never worked with hardware</option>
                            <option value="basic">Basic - Some Arduino/Raspberry Pi</option>
                            <option value="advanced">Advanced - Robotics projects</option>
                        </select>
                    </div>

                    <div className={styles.formGroup}>
                        <label>Programming Languages (comma separated)</label>
                        <input
                            type="text"
                            name="programming_languages"
                            value={formData.programming_languages}
                            onChange={handleChange}
                            placeholder="e.g., Python, C++, JavaScript"
                        />
                    </div>

                    <div className={styles.formGroup}>
                        <label>Robotics Experience</label>
                        <select
                            name="robotics_experience"
                            value={formData.robotics_experience}
                            onChange={handleChange}
                        >
                            <option value="none">None - Complete beginner</option>
                            <option value="beginner">Beginner - Learning basics</option>
                            <option value="intermediate">Intermediate - Built some robots</option>
                            <option value="advanced">Advanced - Professional experience</option>
                        </select>
                    </div>

                    <div className={styles.formGroup}>
                        <label>Learning Goals (Optional)</label>
                        <textarea
                            name="learning_goals"
                            value={formData.learning_goals}
                            onChange={handleChange}
                            placeholder="What do you want to achieve with Physical AI?"
                            rows="3"
                        />
                    </div>

                    <button
                        type="submit"
                        className={styles.submitBtn}
                        disabled={loading}
                    >
                        {loading ? 'Creating Account...' : 'Create Account'}
                    </button>
                </form>
            </div>
        </div>,
        document.body
    );
};

export default SignupPage;
