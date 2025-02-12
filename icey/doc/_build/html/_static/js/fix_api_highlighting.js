/// This is a fix for the issue that the highlight using pygments does not work only because the parent CSS class "highlight" is missing. 
/// "cpp" is just some class that seemingly all API elements have, not 100% sure if it is the correct one though.
/// This is a bug in breathe.
document.addEventListener('DOMContentLoaded', () => {
    document.querySelectorAll('dt.cpp').forEach(element => {
        element.classList.add('highlight');
    });  
    document.querySelectorAll('dl.cpp').forEach(element => {
        /// Simple also removes the margins that dl's by default have in the pydata theme 
        element.classList.add('simple');
    });  
});