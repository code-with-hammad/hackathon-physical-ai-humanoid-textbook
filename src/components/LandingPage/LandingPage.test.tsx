// src/components/LandingPage/LandingPage.test.tsx
import React from 'react';
import { render, screen } from '@testing-library/react';
import LandingPage from './LandingPage';

describe('LandingPage', () => {
  it('should render the book title', () => {
    render(<LandingPage />);
    // This test will fail initially because the title is not yet rendered
    expect(screen.getByText(/AI-Spec–Driven Technical Book/i)).toBeInTheDocument();
  });
});