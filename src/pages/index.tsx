// src/pages/index.tsx
import React from 'react';
import Layout from '@theme/Layout';
import LandingPage from '../components/LandingPage/LandingPage';

export default function Home(): JSX.Element {
  return (
    <Layout
      title="Welcome"
      description="A premium landing page for AI-Spec–Driven Technical Book on Physical AI & Humanoid Robotics with Integrated RAG Chatbot">
      <main>
        <LandingPage />
      </main>
    </Layout>
  );
}
