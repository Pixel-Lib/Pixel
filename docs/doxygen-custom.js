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