/* Add custom JavaScript here */

// Example: Add a console log to check if the file is loaded
console.log("doxygen-custom.js loaded");

// Add a function to change the active tab on click
document.querySelectorAll('.navbar a').forEach((tab) => {
    tab.addEventListener('click', (event) => {
        document.querySelectorAll('.navbar a').forEach((tab) => {
            tab.classList.remove('active');
        });
        event.target.classList.add('active');
    });
});

// Add a function to add a smooth scrolling effect when clicking on a navbar link
document.querySelectorAll('.navbar a').forEach((link) => {
    link.addEventListener('click', (event) => {
        event.preventDefault();
        document.querySelector(event.target.getAttribute('href')).scrollIntoView({
            behavior: 'smooth'
        });
    });
});

// Add a function to toggle a dropdown menu when it's clicked
document.querySelectorAll('.dropdown-toggle').forEach((dropdown) => {
    dropdown.addEventListener('click', (event) => {
        event.preventDefault();
        event.target.nextElementSibling.classList.toggle('show');
    });
});

// Function to switch to dark mode
function switchToDarkMode() {
    document.documentElement.classList.remove('light');
    document.documentElement.classList.add('dark');
}

// Function to switch to light mode
function switchToLightMode() {
    document.documentElement.classList.remove('dark');
    document.documentElement.classList.add('light');
}

// Function to toggle between dark and light mode
function toggleTheme() {
    if (document.documentElement.classList.contains('dark')) {
        switchToLightMode();
    } else {
        switchToDarkMode();
    }
}

// Add event listener to theme switch button
document.getElementById('theme-switcher').addEventListener('click', toggleTheme);

// Set initial theme based on system preference
if (window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches) {
    switchToDarkMode();
} else {
    switchToLightMode();
}