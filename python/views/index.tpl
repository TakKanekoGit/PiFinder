% include("header.tpl", title="Home")

<div id="error" class="error-message"></div>
<div class="center-align">
    <img id="image" src="" alt="PiFinder Screen" class="pifinder-screen z-depth-2">
</div>
<center>
<table class="grey darken-2 grey-text z-depth-1" style="max-width: 512px;">
<tr>
    <td>
      <i class="material-icons medium">wifi</i>
    </td>
    <td class="grey-text text-lighten-1">{{wifi_mode}} Mode<br>{{network_name}}<br>{{ip}}</td>
    <td><a href="/network" class="grey-text"><i class="material-icons">edit</i></a></td>
</tr>
<tr>
    <td>
      <i class="material-icons medium">{{gps_icon}}</i>
    </td>
    <td class="grey-text text-lighten-1">{{gps_text}}<br>lat: {{lat_text}} / lon: {{lon_text}}</td>
    <td><a href="/gps" class="grey-text"><i class="material-icons">edit</i></a></td>
    <td></td>
</tr>
<tr>
    <td>
      <i class="material-icons medium">{{camera_icon}}</i>
    </td>
    <td class="grey-text text-lighten-1">Sky Position<br>RA: {{ra_text}} / DEC: {{dec_text}}</td>
    <td></td>
</tr>
<tr>
    <td>
      <i class="material-icons medium">sd_card</i>
    </td>
    <td class="grey-text text-lighten-1">Software Version<br>{{software_version}}</td>
    <td></td>
</tr>
</table>
</center>
<script>
function fetchImage() {
    const imageElement = document.getElementById('image');
    fetch("/image?t=" + new Date().getTime())
        .then(response => {
            if (!response.ok) { throw Error(response.statusText); }
            return response.blob();
        })
        .then(imageBlob => {
            let imageObjectURL = URL.createObjectURL(imageBlob);
            imageElement.src = imageObjectURL;
            // When the image can't be fetched, display a static message
            const errorElement = document.getElementById('error');
            errorElement.innerHTML = "";
        })
        .catch(error => {
            console.log(error);
            // When the image can't be fetched, display a static message
            const errorElement = document.getElementById('error');
            errorElement.innerHTML = "PiFinder server is currently unavailable. Please try again later.";
        })
        .finally(() => {
            // Schedule the next fetch operation after 100 milliseconds, whether this operation was successful or not
            setTimeout(fetchImage, 500);
        });
}

// Start the first fetch operation
fetchImage();

</script>

% include("footer.tpl", title="PiFinder UI")

