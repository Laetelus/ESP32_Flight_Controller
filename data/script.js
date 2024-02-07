let isUpdating = false
let updateTimeout // This will be used to manage the pause and resume of automatic updates.

document.addEventListener('DOMContentLoaded', function() {
    // Attach event listeners to increment and decrement buttons
    document.querySelectorAll('.decrement-btn').forEach((button) => {
        button.addEventListener('click', function() {
            // Assuming each button has a data-input-id attribute
            decrementValue(this.getAttribute('data-input-id'))
        })
    })
    document.querySelectorAll('.increment-btn').forEach((button) => {
        button.addEventListener('click', function() {
            // Assuming each button has a data-input-id attribute
            incrementValue(this.getAttribute('data-input-id'))
        })
    })

    // Attach event listener to the "Update PID" button
    document
        .getElementById('update-pid-button')
        .addEventListener('click', function() {
            updatePID()
        })

    // Initial fetch of PID values and start the automatic update loop
    getPID()
    startRealTimeUpdate()
})

function userIsUpdating() {
    isUpdating = true
    clearTimeout(updateTimeout) // Stop the automatic update when the user is making changes.
}

function incrementValue(inputId) {
    userIsUpdating();
    const input = document.getElementById(inputId);
    if (input) {
        let currentStep = getStep(input.getAttribute('data-full-precision') || input.value);
        let currentValue = parseFloat(input.getAttribute('data-full-precision') || input.value);
        let newValue = currentValue + currentStep;
        input.value = newValue.toFixed(countDecimals(input.getAttribute('data-full-precision')));
        input.setAttribute('data-full-precision', newValue.toString());
    }
}

function decrementValue(inputId) {
    userIsUpdating();
    const input = document.getElementById(inputId);
    if (input) {
        let currentStep = getStep(input.getAttribute('data-full-precision') || input.value);
        let currentValue = parseFloat(input.getAttribute('data-full-precision') || input.value);
        let newValue = currentValue - currentStep;
        newValue = newValue < 0 ? 0 : newValue;
        input.value = newValue.toFixed(countDecimals(input.getAttribute('data-full-precision')));
        input.setAttribute('data-full-precision', newValue.toString());
    }
}

function getStep(value) {
    // Get the step based on the number of decimal places in the input value
    let decimalCount = countDecimals(value);
    return decimalCount > 0 ? 1 / Math.pow(10, decimalCount) : 0.01;
}


function countDecimals(value) {
    if (Math.floor(value) === value) return 0;
    let decimalPart = value.toString().split(".")[1];
    return decimalPart ? decimalPart.length : 0;
}



function trimTrailingZeros(value) {
    // Trim unnecessary trailing zeros after decimal point
    return value.replace(/(\.\d*?[1-9])0+$|\.0*$/, '$1');
}


document
    .querySelectorAll('.increment-btn, .decrement-btn')
    .forEach((button) => {
        button.addEventListener('click', function() {
            const inputId = this.getAttribute('data-input-id')
            const delta = this.classList.contains('increment-btn') ? 0.01 : -0.01
            changeValue(inputId, delta)
        })
    })

function changeValue(inputId, delta) {
    const input = document.getElementById(inputId)
    if (input) {
        let value = parseFloat(input.value) || 0
        value = Math.max(value + delta, 0)
        input.value = value.toFixed(2)
    }
}

function handleResponseError(response) {
    if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`)
    }
    return response
}

// Modify this function to check the isUpdating flag
function getPID() {
    if (!isUpdating) {
        // Only fetch and update display if the user is not making changes
        fetch('/getPID')
            .then((response) => response.json())
            .then((data) => {
                updatePIDDisplay(data)
            })
            .catch((error) => {
                console.error('Error:', error)
            })
    }
}

function updatePIDDisplay(data) {
    userIsUpdating()
        //Roll
    document.getElementById('p-gain-roll').value = data.pid_p_gain_roll
    document.getElementById(
        'current-p-gain-roll'
    ).textContent = `(Current: ${data.pid_p_gain_roll})`

    document.getElementById('i-gain-roll').value = data.pid_i_gain_roll
    document.getElementById(
        'current-i-gain-roll'
    ).textContent = `(Current: ${data.pid_i_gain_roll})`

    document.getElementById('d-gain-roll').value = data.pid_d_gain_roll
    document.getElementById(
        'current-d-gain-roll'
    ).textContent = `(Current: ${data.pid_d_gain_roll})`

    //Yaw
    document.getElementById('p-gain-yaw').value = data.pid_p_gain_yaw
    document.getElementById(
        'current-p-gain-yaw'
    ).textContent = `(Current: ${data.pid_p_gain_yaw})`

    document.getElementById('i-gain-yaw').value = data.pid_i_gain_yaw
    document.getElementById(
        'current-i-gain-yaw'
    ).textContent = `(Current: ${data.pid_i_gain_yaw})`

    document.getElementById('d-gain-yaw').value = data.pid_d_gain_yaw
    document.getElementById(
        'current-d-gain-yaw'
    ).textContent = `(Current: ${data.pid_d_gain_yaw}`
}

function updatePID() {
    // Indicate that updates are being processed
    isUpdating = true

    // Prepare your data for sending
    const data = {
        pid_p_gain_roll: document.getElementById('p-gain-roll').value,
        pid_i_gain_roll: document.getElementById('i-gain-roll').value,
        pid_d_gain_roll: document.getElementById('d-gain-roll').value,
        pid_p_gain_yaw: document.getElementById('p-gain-yaw').value,
        pid_i_gain_yaw: document.getElementById('i-gain-yaw').value,
        pid_d_gain_yaw: document.getElementById('d-gain-yaw').value,
    }

    // Convert the data object into a URL-encoded string
    const formData = Object.keys(data)
        .map((key) => `${encodeURIComponent(key)}=${encodeURIComponent(data[key])}`)
        .join('&')

    // Send your data to the server
    fetch('/setPID', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/x-www-form-urlencoded',
            },
            body: formData,
        })
        .then(handleResponseError)
        .then(() => {
            console.log('PID values updated successfully')
                // After updating, resume automatic updates
            setTimeout(() => {
                    isUpdating = false
                    getPID() // Fetch and display updated PID values
                }, 1000) // 1-second delay before resuming automatic updates
        })
        .catch((error) => console.error('Error updating PID:', error))
}

function startRealTimeUpdate() {
    setInterval(getPID, 5000) // Adjust interval as needed
}