import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatBot.module.css';

const ChatBot = () => {
    const [isOpen, setIsOpen] = useState(false);
    const [isMaximized, setIsMaximized] = useState(false);
    const [messages, setMessages] = useState([]);
    const [input, setInput] = useState('');
    const [isLoading, setIsLoading] = useState(false);
    const [sessionId, setSessionId] = useState(null);
    const messagesEndRef = useRef(null);

    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
    };

    useEffect(() => {
        scrollToBottom();
    }, [messages]);

    useEffect(() => {
        const handleOpenChat = (e) => {
            const { selectedText } = e.detail;
            setIsOpen(true);
            if (selectedText) {
                setInput(`Explain this: "${selectedText}"`);
            }
        };

        window.addEventListener('open-chat', handleOpenChat);
        return () => window.removeEventListener('open-chat', handleOpenChat);
    }, []);

    const sendMessage = async () => {
        if (!input.trim()) return;

        const userMessage = input.trim();
        setInput('');

        setMessages(prev => [...prev, { role: 'user', content: userMessage }]);
        setIsLoading(true);

        try {
            const response = await fetch('http://localhost:8000/chat', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    message: userMessage,
                    session_id: sessionId
                })
            });

            const data = await response.json();

            if (response.ok) {
                setSessionId(data.session_id);
                setMessages(prev => [...prev, {
                    role: 'assistant',
                    content: data.response,
                    sources: data.sources
                }]);
            } else {
                setMessages(prev => [...prev, {
                    role: 'assistant',
                    content: 'Sorry, I encountered an error. Please try again.'
                }]);
            }
        } catch (error) {
            setMessages(prev => [...prev, {
                role: 'assistant',
                content: 'Failed to connect to the chatbot. Make sure the backend is running.'
            }]);
        } finally {
            setIsLoading(false);
        }
    };

    const handleKeyPress = (e) => {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            sendMessage();
        }
    };

    return (
        <>
            <button
                className={styles.chatButton}
                onClick={() => setIsOpen(!isOpen)}
                aria-label="Open chat"
            >
                {isOpen ? '✕' : '💬'}
            </button>

            {isOpen && (
                <div className={`${styles.chatWindow} ${isMaximized ? styles.maximized : ''}`}>
                    <div className={styles.chatHeader}>
                        <div>
                            <h3>Physical AI Assistant</h3>
                            <p>Ask me anything about ROS 2, Gazebo, or VLA!</p>
                        </div>
                        <button
                            className={styles.maximizeBtn}
                            onClick={() => setIsMaximized(!isMaximized)}
                            aria-label={isMaximized ? "Minimize" : "Maximize"}
                        >
                            {isMaximized ? '⊟' : '⊡'}
                        </button>
                    </div>

                    <div className={styles.chatMessages}>
                        {messages.length === 0 && (
                            <div className={styles.welcomeMessage}>
                                <p>👋 Hi! I'm your Physical AI assistant.</p>
                                <p>Ask me about:</p>
                                <ul>
                                    <li>ROS 2 concepts</li>
                                    <li>Gazebo simulation</li>
                                    <li>NVIDIA Isaac</li>
                                    <li>Vision-Language-Action models</li>
                                </ul>
                            </div>
                        )}

                        {messages.map((msg, idx) => (
                            <div
                                key={idx}
                                className={`${styles.message} ${styles[msg.role]}`}
                            >
                                <div className={styles.messageContent}>
                                    {msg.content}
                                </div>
                                {msg.sources && (
                                    <div className={styles.sources}>
                                        Sources: {msg.sources.join(', ')}
                                    </div>
                                )}
                            </div>
                        ))}

                        {isLoading && (
                            <div className={`${styles.message} ${styles.assistant}`}>
                                <div className={styles.typing}>
                                    <span></span>
                                    <span></span>
                                    <span></span>
                                </div>
                            </div>
                        )}

                        <div ref={messagesEndRef} />
                    </div>

                    <div className={styles.chatInput}>
                        <textarea
                            value={input}
                            onChange={(e) => setInput(e.target.value)}
                            onKeyPress={handleKeyPress}
                            placeholder="Ask a question..."
                            rows="2"
                        />
                        <button
                            onClick={sendMessage}
                            disabled={isLoading || !input.trim()}
                        >
                            Send
                        </button>
                    </div>
                </div>
            )}
        </>
    );
};

export default ChatBot;
