using LibSerialPort
using Base64
# include("pendulum_pb.jl")  # Protocol Buffers

"Find USB port connected to Teensy"
function get_teensy_port_name(;nports_guess::Integer=64)
  ports = sp_list_ports()

  teensy_port_name = ""

  for port in unsafe_wrap(Array, ports, nports_guess, own=false)
    port == C_NULL && break
    if sp_get_port_transport(port) == SP_TRANSPORT_USB
      if sp_get_port_usb_manufacturer(port) == "Teensyduino" || sp_get_port_usb_vid_pid(port) == (5824, 1155)
          teensy_port_name = sp_get_port_name(port)
          break
      end
    end
  end

  sp_free_port_list(ports)
  return teensy_port_name
end

"Open serial connectoin on USB port connected to Teensy"
function open_teensy_port()
  portname = get_teensy_port_name()

  if portname == ""
    @warn "Could not find Teensy USB port"
    return nothing
  end

  open(portname,12000000)
end

mutable struct MessageCollector
  in_message::Bool
  buf::IOBuffer
  decode_buf::Base64DecodePipe
end
MessageCollector() = ( io = IOBuffer(); MessageCollector(false, io, Base64DecodePipe(io)) )

"""Receive available bytes from `sp` and collect those belonging to messages in `mc`.
Return buffer length if a message was completely received.
Return value of 0 could mean that `mc` contains a partially buffered message.
"""
function collect_message!(mc::MessageCollector, sp::SerialPort)
  while bytesavailable(sp) > 0
    b = read(sp, Char)
    if b == '['                 # start of message
      take!(mc.buf)             # clear buffer
      mc.in_message = true
      # new message -> keep receiving
    elseif mc.in_message
      if b == ']'               # end of message
        mc.in_message = false
        # message complete -> return for parsing
        return mc.buf.size
      else                      # write byte to message buffer
        write(mc.buf, b)
        # partial message -> keep receiving
      end
    end
  end
  return 0 # neither bytes available nor complete message collected
end

function get_message(mc::MessageCollector, ::Type{T}) where T<:ProtoType
  seekstart(mc.buf)
  readproto(mc.decode_buf, ProtoBuf.instantiate(T))
end

function send_message(msg::ProtoType, sp::SerialPort)
  msg_b64 = string("[", base64encode(writeproto, msg), "]")
  write(sp, msg_b64)
end

function drain_input(sp::SerialPort, idle_ms::Int=100, timeout_ms::Int=1000)
  start_time = time_ns()
  idle_time = start_time
  while !eof(sp)
    if (time_ns() - start_time)/1e6 > timeout_ms
      break
    end
    if bytesavailable(sp) > 0
      idle_time = time_ns()
      read(sp, Char)
    elseif (time_ns() - idle_time)/1e6 > idle_ms
      break
    end
  end
end
