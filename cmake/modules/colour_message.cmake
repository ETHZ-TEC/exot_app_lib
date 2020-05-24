# Copyright (c) 2015-2020, Swiss Federal Institute of Technology (ETH Zurich)
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
##
# @file colour_message.cmake
# @author Bruno Klopott
# @brief Function to print a coloured message to the console
#

string(ASCII 27 Escape)
set(Reset       "${Escape}[m")
set(Bold        "${Escape}[1m")
set(Red         "${Escape}[31m")
set(Green       "${Escape}[32m")
set(Yellow      "${Escape}[33m")
set(Blue        "${Escape}[34m")
set(Magenta     "${Escape}[35m")
set(Cyan        "${Escape}[36m")
set(White       "${Escape}[37m")

function(colour_message)
	list(GET ARGV 0 message_type)
	list(GET ARGV 1 colour)

	list(GET ARGV 2 bold)

	if (${bold} STREQUAL BOLD)
		set(use_bold ON)
		list(REMOVE_AT ARGV 2)
	endif ()

	list(REMOVE_AT ARGV 1)
	list(REMOVE_AT ARGV 0)

	if (${colour}     STREQUAL RED)
		if (${use_bold})
			set(modifier "${Bold}${Red}")
		else ()
			set(modifier "${Red}")
		endif ()
	elseif (${colour} STREQUAL GREEN)
		if (${use_bold})
			set(modifier "${Bold}${Green}")
		else ()
			set(modifier "${Green}")
		endif ()
	elseif (${colour} STREQUAL YELLOW)
		if (${use_bold})
			set(modifier "${Bold}${Yellow}")
		else ()
			set(modifier "${Yellow}")
		endif ()
	elseif (${colour} STREQUAL BLUE)
		if (${use_bold})
			set(modifier "${Bold}${Blue}")
		else ()
			set(modifier "${Blue}")
		endif ()
	elseif (${colour} STREQUAL MAGENTA)
		if (${use_bold})
			set(modifier "${Bold}${Magenta}")
		else ()
			set(modifier "${Magenta}")
		endif ()
	elseif (${colour} STREQUAL CYAN)
		if (${use_bold})
			set(modifier "${Bold}${Cyan}")
		else ()
			set(modifier "${Cyan}")
		endif ()
	elseif (${colour} STREQUAL WHITE)
		if (${use_bold})
			set(modifier "${Bold}${White}")
		else ()
			set(modifier "${White}")
		endif ()
	else (${colour})
		if (${use_bold})
			set(modifier "${Bold}")
		else ()
			set(modifier "")
		endif ()
	endif ()

	message(${message_type} "${modifier}${ARGV}${Reset}")
endfunction()
