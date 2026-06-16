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

function stripTrailingZeros(str) {
    return str
        .replace(/(\.\d*?[1-9])0+$/, '$1')
        .replace(/\.0+$/, '')
}

function formatPID(val) {
    if (!Number.isFinite(val)) return ''
    const clamped = clampValue(val)
    return stripTrailingZeros(String(clamped))
}

function normalizeIncomingValue(value) {
    if (value === null || value === undefined) return ''
    const text = String(value).trim()
    if (text === '') return ''

    const num = Number(text)
    if (!Number.isFinite(num)) return ''

    return formatPID(num)
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
    const formatted = normalizeIncomingValue(value)
    const input = document.getElementById(id)
    const current = document.getElementById('current-' + id)

    current.textContent = formatted === '' ? '' : `(Current: ${formatted})`

    // Keep editable inputs empty until the user enters/adjusts a value.
    // This prevents the UI from auto-filling 0 or poll values into the boxes.
    if (input.value.trim() === '') {
        return
    }

    // If user already has a value in the input, keep it synced with backend value.
    input.value = formatted
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

    // Prepare only fields the user actually filled in.
    const fieldMap = {
        pid_p_gain_roll: 'p-gain-roll',
        pid_i_gain_roll: 'i-gain-roll',
        pid_d_gain_roll: 'd-gain-roll',
        pid_p_gain_yaw: 'p-gain-yaw',
        pid_i_gain_yaw: 'i-gain-yaw',
        pid_d_gain_yaw: 'd-gain-yaw',
    }

    const data = {}
    Object.keys(fieldMap).forEach((key) => {
        const raw = document.getElementById(fieldMap[key]).value.trim()
        if (raw !== '') {
            data[key] = raw
        }
    })

    if (Object.keys(data).length === 0) {
        showToast('\u2717 No PID fields entered.', true)
        isUpdating = false
        return
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