# Function to add a port forwarding rule
function Add-PortForwarding {
    param (
        [string]$ListenAddress,
        [int]$ListenPort,
        [string]$ConnectAddress,
        [int]$ConnectPort
    )
    Write-Host "Adding port forwarding: Listen on $ListenAddress:$ListenPort -> Connect to $ConnectAddress:$ConnectPort"
    netsh interface portproxy add v4tov4 listenaddress=$ListenAddress listenport=$ListenPort connectaddress=$ConnectAddress connectport=$ConnectPort
}

# Function to remove a specific port forwarding rule
function Remove-PortForwarding {
    param (
        [int]$Port
    )
    Write-Host "Removing port forwarding for port $Port..."
    netsh interface portproxy delete v4tov4 listenport=$Port
}

# Function to show all configured port forwarding rules
function Show-PortForwarding {
    Write-Host "Current port forwarding rules:"
    netsh interface portproxy show v4tov4
}

# Function to reset all port forwarding rules
function Reset-PortForwarding {
    Write-Host "Removing all port forwarding rules..."
    netsh interface portproxy reset
}

# Function to automatically add port forwarding for Python Web and WebSocket
function Add-AutoForwarding {
    param (
        [string]$ListenAddress,
        [string]$ConnectAddress
    )
    Write-Host "Adding automatic port forwarding for Python Web and WebSocket..."
    Add-PortForwarding -ListenAddress $ListenAddress -ListenPort 8000 -ConnectAddress $ConnectAddress -ConnectPort 8000
    Add-PortForwarding -ListenAddress $ListenAddress -ListenPort 9090 -ConnectAddress $ConnectAddress -ConnectPort 9090
    Add-PortForwarding -ListenAddress $ListenAddress -ListenPort 11311 -ConnectAddress $ConnectAddress -ConnectPort 11311
}

# User menu
while ($true) {
    Write-Host "\nPort Forwarding Manager"
    Write-Host "--------------------------------"
    Write-Host "1. Add port forwarding"
    Write-Host "2. Add automatic forwarding"
    Write-Host "3. Show current port forwarding rules"
    Write-Host "4. Remove specific port forwarding"
    Write-Host "5. Reset all port forwarding rules"
    Write-Host "6. Exit"

    $choice = Read-Host "Enter your choice"

    switch ($choice) {
        1 {
            $listenAddress = Read-Host "Enter the listen address (i.e Windows):"
            $listenPort = Read-Host "Enter the listen port" | ForEach-Object { [int]$_ }
            $connectAddress = Read-Host "Enter the connect address (i.e WSL2):"
            $connectPort = Read-Host "Enter the connect port" | ForEach-Object { [int]$_ }
            Add-PortForwarding -ListenAddress $listenAddress -ListenPort $listenPort -ConnectAddress $connectAddress -ConnectPort $connectPort
        }
        2 {
            $listenAddress = Read-Host "Enter the listen address (i.e Windows):"
            $connectAddress = Read-Host "Enter the connect address (i.e WSL2):"
            Add-AutoForwarding -ListenAddress $listenAddress -ConnectAddress $connectAddress
        }
        3 {
            Show-PortForwarding
        }
        4 {
            $port = Read-Host "Enter the port to remove" | ForEach-Object { [int]$_ }
            Remove-PortForwarding -Port $port
        }
        5 {
            Reset-PortForwarding
        }
        6 {
            Write-Host "Exiting..."
            break
        }
        default {
            Write-Host "Invalid choice. Please select a valid option."
        }
    }
}
