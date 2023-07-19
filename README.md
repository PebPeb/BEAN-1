# BEAN-1 (RV32I)

The BEAN-1 is a Von-Neumann style CPU that executes the RV32I instruction set. The CPU has two states, a fetch and an execute state. During the fetch state the CPU will load the next instruction from memory. In the execute state, the CPU executes the fetched instruction. This system is designed with a shared data bus between the memory and the datapath.


![BEAN-1 Datapath](assets/BEAN-1.png)

