$(function() {
            var $items = $('#vtab>ul>li');
            $items.mousedown(function() {
                $items.removeClass('selected');
                $(this).addClass('selected');

                var index = $items.index($(this));
                $('#vtab>div').hide().eq(index).show();
            }).eq(0).mousedown();
});


$(function(){
        $('.slide-out-div').tabSlideOut({
            tabHandle: '.handle',                     //class of the element that will become your tab
            pathToTabImage: 'images/robot_view_tab.png', //path to the image for the tab //Optionally can be set using css
            imageHeight: '132px',                     //height of tab image           //Optionally can be set using css
            imageWidth: '44px',                       //width of tab image            //Optionally can be set using css
            tabLocation: 'right',                      //side of screen where tab lives, top, right, bottom, or left
            speed: 300,                               //speed of animation
            action: 'click',                          //options: 'click' or 'hover', action to trigger animation
            topPos: '500px',                          //position from the top/ use if tabLocation is left or right
            leftPos: '20px',                          //position from left/ use if tabLocation is bottom or top
            fixedPosition: true                      //options: true makes it stick(fixed position) on scroll
        });
});