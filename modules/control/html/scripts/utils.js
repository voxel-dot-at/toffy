/**
 * local utility scripts and functions
*/

//> split of eventually existing page parameters
var params = {};

var splitUrlParameters = function getUrlParameter() {
    var sPageURL = decodeURIComponent(window.location.search.substring(1)),
        sURLVariables = sPageURL.split('&'),
        paramName,
        i;

    for (i = 0; i < sURLVariables.length; i++) {
        paramName = sURLVariables[i].split('=');
	params[ paramName[0] ] = paramName[1];
    }
}


/** construct the URL to the filter details page.
 * 
 */
var getFilterDetailsUrl = function(data, filter)
{
    var uri = null;
    if (data.type == "bta")
        uri = "/ui/filter/bta.html";
    else if (data.type == "sensor2d")
        uri = "/ui/filter/sensor2d.html";
    else if (data.type == "sensorUsb")
        uri = "/ui/filter/sensor2d.html";
    else if (data.type == "transform")
        uri = "/ui/filter/transform.html";

    if (uri !== null) {	
	uri+= "?id=" + filter;
	//console.log("uri: " + uri );
    } else {
	//console.log("uri: no special type found. returning null! " );
    }
    return uri;
}
