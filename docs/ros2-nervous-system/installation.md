# Installation Instructions

This section provides detailed instructions for setting up your development environment to follow along with the book. You will need to install ROS 2, Python, and other dependencies.

## 1. Install ROS 2

ROS 2 (Robot Operating System 2) is the core middleware used throughout this book. Please follow the official installation guide for your operating system:

*   **Ubuntu (recommended)**: [ROS 2 Documentation: Install ROS 2 on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
*   **Windows**: [ROS 2 Documentation: Install ROS 2 on Windows](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)
*   **macOS**: [ROS 2 Documentation: Install ROS 2 on macOS](https://docs.ros.org/en/humble/Installation/macOS-Install-Binary.html)

**Important**: After installation, remember to source your ROS 2 environment in every new terminal session or add it to your shell's startup file (e.g., `.bashrc`, `.zshrc`).

```bash
source /opt/ros/<ros2-distro>/setup.bash
```
Replace `<ros2-distro>` with your installed ROS 2 distribution (e.g., `humble`).

## 2. Install Python

This book primarily uses Python 3.11. Ensure you have Python 3.8 or newer installed. Most modern operating systems come with Python pre-installed. You can check your Python version:

```bash
python3 --version
```

If you need to install or update Python, refer to the official Python documentation or use your system's package manager.

## 3. Install Docusaurus Dependencies

The book content is presented using Docusaurus. To set up the Docusaurus development environment, follow these steps:

### Node.js and Yarn

Docusaurus requires Node.js (version 18.0 or later) and Yarn (version 1.5 or later).

1.  **Install Node.js**: Download from [nodejs.org](https://nodejs.org/en/).
2.  **Install Yarn**:
    ```bash
    npm install -g yarn
    ```

### Project Dependencies

1.  **Clone the book repository:**
    ```bash
    git clone <repository-url>
    cd <repository-directory>
    ```
2.  **Install project-specific dependencies:**
    ```bash
    yarn install
    ```

## 4. Install Python Dependencies

Some code examples will require specific Python libraries. These can typically be installed using `pip`:

```bash
pip install rclpy # Example: for ROS 2 Python client library
```

More specific Python dependencies will be listed in the respective chapters.
