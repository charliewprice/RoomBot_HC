/*
 * ErriezSerialTerminal.cpp
 *
 *  Created on: Nov 11, 2021
 *      Author: Charlie
 */

/*
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*!
 * \file ErriezSerialTerminal.cpp
 * \brief Serial terminal library for Arduino.
 * \details
 *      Source:         https://github.com/Erriez/ErriezSerialTerminal
 *      Documentation:  https://erriez.github.io/ErriezSerialTerminal
 */

#include "ErriezSerialTerminal.h"



/*!
 * \brief SerialTerminal constructor.
 * \param newlineChar
 *      Newline character \c '\\r' or \c '\\n'. \n
 *      Default: \c '\\n'.
 * \param delimiterChar
 *      Delimiter separator character between commands and arguments.\n
 *      Default: space.
 */
SerialTerminal::SerialTerminal(char newlineChar, char delimiterChar) :
        _commandList(NULL),
        _numCommands(0),
        _newlineChar(newlineChar),
        _lastPos(NULL),
        _defaultHandler(NULL)

{
    // strtok_r needs a null-terminated string
    _delimiter[0] = delimiterChar;
    _delimiter[1] = '\0';

    // Clear receive buffer
    clearBuffer();
}


/*!
 * \brief Add command with callback handler.
 * \param command
 *      Register a null-terminated ASCII command.
 * \param function
 *      The function to be called when receiving the \c command.
 */

void SerialTerminal::addCommand(const char *command, void (*function)())
{
    // Increase size command list by one
    _commandList = (SerialTerminalCallback *)realloc(_commandList,
            sizeof(SerialTerminalCallback) * (_numCommands + 1));

    // Copy command and store command callback handler
    strncpy(_commandList[_numCommands].command, command, ST_NUM_COMMAND_CHARS);
    _commandList[_numCommands].function = function;

    // Increment number of commands
    _numCommands++;
}


/*!
 * \brief Set the control state to echo any printable chars to the console.
 * \details
 *      Set the control state to echo any printable chars to the console.
 * \param doEcho
 *      Should all printable chars be echoed to the serial console?
 */
void SerialTerminal::setSerialEcho(bool doEcho)
{
    doCharEcho = doEcho;
}

/*!
 * \brief Set post command handler callback for after all handled commands.
 * \details
 *      Store post command handler which will be called when after executing any handled command.
 * \param function
 *      Address of the callback function. This function will be called after any handled event.
 */
void SerialTerminal::setPostCommandHandler(void (*function)(void))
{
    _postCommandHandler = function;
}

/*!
 * \brief Set default callback handler for unknown commands.
 * \details
 *      Store default callback handler which will be called when receiving an unknown command.
 * \param function
 *      Address of the default handler. This function will be called when the command is not
 *      recognized. The parameter contains the first ASCII command.
 */
void SerialTerminal::setDefaultHandler(void (*function)(const char *))
{
    _defaultHandler = function;
}

/*!
 * \brief Read from serial port.
 * \details
 *      Process command when newline character has been received.
 */
void SerialTerminal::readSerial()
{
    bool matched = false;
    char *command;
    char c;
    //SerialPort.print(".");
    while (SerialPort.available() > 0) {
        // Read one character from serial port
        c = (char)SerialPort.read();

        // Check newline character \c '\\r' or \c '\\n' at the end of a command
        if (c == _newlineChar) {
            //Echo received char
            if (doCharEcho) {
                SerialPort.println();
            }

            // Search for command at start of buffer
            command = strtok_r(_rxBuffer, _delimiter, &_lastPos);

            if (command != NULL) {
                for (int i = 0; i < _numCommands; i++) {
                    // Compare the found command against the list of known commands for a match
                    if (strncmp(command, _commandList[i].command, ST_NUM_COMMAND_CHARS) == 0) {
                        // Call command callback handler
                        (*_commandList[i].function)();
                        matched = true;
                        break;
                    }
                }

                if (!matched && (_defaultHandler != NULL)) {
                    (*_defaultHandler)(command);
                }
            }

            //Run the post command handler.
            if (_postCommandHandler) {
                (*_postCommandHandler)();
            }

            clearBuffer();
        // either ^H og 127 backspace chars
        } else if (c == '\b' || c == 127) {
            if (_rxBufferIndex > 0) {
                _rxBufferIndex--;
                _rxBuffer[_rxBufferIndex] = '\0';
                if (doCharEcho) {
                    SerialPort.print("\b \b"); // 1 char back, space, 1 char back
                }
            }
        } else if (isprint(c)) {
            // Store printable characters in serial receive buffer
            if (_rxBufferIndex < ST_RX_BUFFER_SIZE) {
                _rxBuffer[_rxBufferIndex++] = c;
                _rxBuffer[_rxBufferIndex] = '\0';
                //Echo received char
                if (doCharEcho) {
                    SerialPort.print(c);
                }
            }
        }
    }
}

/*!
 * \brief Clear serial receive buffer.
 */
void SerialTerminal::clearBuffer()
{
    _rxBuffer[0] = '\0';
    _rxBufferIndex = 0;
}

/*!
 * \brief Get next argument.
 * \return
 *      Address: Pointer to next argument\n
 *      NULL: No argument available
 */
char *SerialTerminal::getNext()
{
    return strtok_r(NULL, _delimiter, &_lastPos);
}

/*!
 * \brief Get all remaining characters from serial buffer.
 * \return
 *      Address: Pointer to remaining characters in serial receive buffer.\n
 *      NULL: No remaining data available.
 */
char *SerialTerminal::getRemaining()
{
    return strtok_r(NULL, "", &_lastPos);
}
