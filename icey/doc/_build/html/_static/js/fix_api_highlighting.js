document.addEventListener('DOMContentLoaded', () => {
/// This is a fix for the issue that the highlight using pygments does not work because the parent CSS class highlight is missing. 
/// "cpp" is just some class that seemingly all API elements have, not 100% sure if it is the correct one tho
document.querySelectorAll('.cpp').forEach(element => {
    console.log("Called HIGHGlight fixed !");
    element.classList.add('highlight');
 });  
});