let isUpdating = false
let updateTimeout // This will be used to manage the pause and resume of automatic updates.

document.addEventListener('DOMContentLoaded', function() {
    // Attach event listeners to increment and decrement buttons
    document.querySelectorAll('.decrement-btn').forEach((button) => {
        button.addEventListener('click', function() {
            decrementValue(this.getAttribute('data-input-id'))
        })
    })

    document.querySelectorAll('.increment-btn').forEach((button) => {
        button.addEventListener('click', function() {
            incrementValue(this.getAttribute('data-input-id'))
        })
    })

    // Attach event listener to the "Update PID" button
    document
        .getElementById('update-pid-button')
        .addEventListener('click', function() {
            updatePID()
        })

    // Attach event listeners to input fields to capture "Enter" (Return) key and focus
    document.querySelectorAll('input').forEach((input) => {
        // Detect "Enter" (Return) key press on both desktop and mobile (iOS/Android) virtual keyboards
        input.addEventListener('keypress', function(event) {
            if (event.key === 'Enter') {
                event.preventDefault() // Prevent default form submission
                updatePID() // Trigger PID update when "Return" is pressed
                input.blur() // Close virtual keyboard on mobile
            }
        })

        // Pause automatic updates when the user focuses on an input field
        input.addEventListener('focus', function() {
            userIsUpdating()
        })

        // On blur, wait briefly then check if another input got focus
        input.addEventListener('blur', function() {
            clearTimeout(updateTimeout)
            updateTimeout = setTimeout(function() {
                // If no input is focused after the delay, resume auto-refresh
                if (!document.activeElement || document.activeElement.tagName !== 'INPUT') {
                    isUpdating = false
                }
            }, 200)
        })
    })

    // Initial fetch of PID values and start the automatic update loop
    getPID()
    startRealTimeUpdate()
})

function userIsUpdating() {
    isUpdating = true
    clearTimeout(updateTimeout) // Stop the automatic update when the user is making changes.
}

function clampValue(val) {
    if (val < 0) return 0
    if (val > 99.999) return 99.999
    return val
}

function formatPID(val) {
    return clampValue(val).toFixed(3)
}

function incrementValue(inputId) {
    userIsUpdating()
    const input = document.getElementById(inputId)
    if (input) {
        let currentValue = parseFloat(input.value) || 0
        let newValue = currentValue + 0.001
        input.value = formatPID(newValue)
    }
}

function decrementValue(inputId) {
    userIsUpdating()
    const input = document.getElementById(inputId)
    if (input) {
        let currentValue = parseFloat(input.value) || 0
        let newValue = currentValue - 0.001
        input.value = formatPID(newValue)
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

function setField(id, value) {
    let v = parseFloat(value) || 0
    let formatted = formatPID(v)
    document.getElementById(id).value = formatted
    document.getElementById('current-' + id).textContent = `(Current: ${formatted})`
}

function updatePIDDisplay(data) {
    // Roll
    setField('p-gain-roll', data.pid_p_gain_roll)
    setField('i-gain-roll', data.pid_i_gain_roll)
    setField('d-gain-roll', data.pid_d_gain_roll)
    // Yaw
    setField('p-gain-yaw', data.pid_p_gain_yaw)
    setField('i-gain-yaw', data.pid_i_gain_yaw)
    setField('d-gain-yaw', data.pid_d_gain_yaw)
}

function showToast(message, isError) {
    let toast = document.getElementById('toast-notification')
    if (!toast) {
        toast = document.createElement('div')
        toast.id = 'toast-notification'
        document.body.appendChild(toast)
    }
    toast.textContent = message
    toast.className = 'toast ' + (isError ? 'toast-error' : 'toast-success')
    toast.style.opacity = '1'
    clearTimeout(toast._hideTimeout)
    toast._hideTimeout = setTimeout(() => { toast.style.opacity = '0' }, 3000)
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
            headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
            body: formData,
        })
        .then((response) => response.text().then((text) => ({ ok: response.ok, text })))
        .then(({ ok, text }) => {
            if (ok) {
                showToast('\u2713 ' + text, false)
                setTimeout(() => { isUpdating = false; getPID() }, 1000)
            } else {
                showToast('\u2717 ' + text, true)
                isUpdating = false
            }
        })
        .catch((error) => {
            showToast('\u2717 Network error: ' + error.message, true)
            isUpdating = false
        })
}

function startRealTimeUpdate() {
    setInterval(getPID, 5000) // Adjust interval as needed
}