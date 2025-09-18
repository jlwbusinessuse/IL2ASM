using dnlib.DotNet;
using dnlib.DotNet.Emit;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.Intrinsics.Arm;

namespace IL2ASM
{
    internal class Program
    {
        static void Main(string[] args)
        {
            if (args.Length < 2)
            {
                Console.WriteLine("Usage: IL2ASM <input.dll/exe> <architecture> [output.asm]");
                Console.WriteLine("Supported architectures: x64, x86, arm64, arm32");
                return;
            }

            string inputFile = args[0];
            string architecture = args[1].ToLower();
            string outputFile = args.Length > 2 ? args[2] : Path.ChangeExtension(inputFile, ".asm");

            try
            {
                var module = ModuleDefMD.Load(inputFile);
                string assemblyCode = "";

                switch (architecture)
                {
                    case "x64":
                        assemblyCode = ConvertToX64(module);
                        break;
                    case "x86":
                        assemblyCode = ConvertToX86(module);
                        break;
                    case "arm64":
                        assemblyCode = ConvertToARM64(module);
                        break;
                    case "arm32":
                        assemblyCode = ConvertToARM32(module);
                        break;
                    default:
                        Console.WriteLine($"Unsupported architecture: {architecture}");
                        return;
                }

                File.WriteAllText(outputFile, assemblyCode);
                Console.WriteLine($"Successfully converted to {architecture} assembly: {outputFile}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error: {ex.Message}");
            }
        }

        private static string ConvertToX64(ModuleDef module)
        {
            var assemblyBuilder = new AssemblyBuilder("x64");
            ProcessModule(module, assemblyBuilder);
            return assemblyBuilder.GenerateAssembly();
        }

        private static string ConvertToX86(ModuleDef module)
        {
            var assemblyBuilder = new AssemblyBuilder("x86");
            ProcessModule(module, assemblyBuilder);
            return assemblyBuilder.GenerateAssembly();
        }

        private static string ConvertToARM64(ModuleDef module)
        {
            var assemblyBuilder = new AssemblyBuilder("arm64");
            ProcessModule(module, assemblyBuilder);
            return assemblyBuilder.GenerateAssembly();
        }

        private static string ConvertToARM32(ModuleDef module)
        {
            var assemblyBuilder = new AssemblyBuilder("arm32");
            ProcessModule(module, assemblyBuilder);
            return assemblyBuilder.GenerateAssembly();
        }

        private static void ProcessModule(ModuleDef module, AssemblyBuilder builder)
        {
            // Process all types in the module
            foreach (var type in module.Types)
            {
                if (type.IsGlobalModuleType)
                    continue;

                ProcessType(type, builder);
            }
        }

        private static void ProcessType(TypeDef type, AssemblyBuilder builder)
        {
            // Track attributes
            var attributeTracker = new AttributeTracker(type);
            builder.AddComment($"Type: {type.FullName}");

            // Process methods
            foreach (var method in type.Methods)
            {
                if (!method.HasBody)
                    continue;

                ProcessMethod(method, builder, attributeTracker);
            }

            // Process nested types
            foreach (var nestedType in type.NestedTypes)
            {
                ProcessType(nestedType, builder);
            }
        }

        private static void ProcessMethod(MethodDef method, AssemblyBuilder builder, AttributeTracker attributeTracker)
        {
            builder.AddComment($"Method: {method.FullName}");

            // Apply any method-specific attributes
            attributeTracker.TrackMethodAttributes(method, builder);

            // Add method prologue
            builder.AddMethodPrologue(method);

            // Process IL instructions
            foreach (var instruction in method.Body.Instructions)
            {
                ConvertInstruction(instruction, builder);
            }

            // Add method epilogue
            builder.AddMethodEpilogue(method);
        }

        private static void ConvertInstruction(Instruction instruction, AssemblyBuilder builder)
        {
            // Convert IL instruction to target architecture assembly
            builder.ConvertILInstruction(instruction);
        }
    }

    // Class to track and process .NET attributes
    public class AttributeTracker
    {
        private readonly TypeDef _type;
        private readonly Dictionary<string, object> _typeAttributes = new Dictionary<string, object>();

        public AttributeTracker(TypeDef type)
        {
            _type = type;
            ProcessTypeAttributes();
        }

        private void ProcessTypeAttributes()
        {
            if (!_type.HasCustomAttributes)
                return;

            foreach (var attr in _type.CustomAttributes)
            {
                // Track type attributes
                _typeAttributes[attr.TypeFullName] = GetAttributeValue(attr);
            }
        }

        public void TrackMethodAttributes(MethodDef method, AssemblyBuilder builder)
        {
            if (!method.HasCustomAttributes)
                return;

            foreach (var attr in method.CustomAttributes)
            {
                var value = GetAttributeValue(attr);
                // Add method attribute specific assembly directives
                builder.AddDirective($"// Attribute: {attr.TypeFullName}");
            }
        }

        private object GetAttributeValue(CustomAttribute attr)
        {
            // Simple implementation - would need to be expanded based on attribute types
            return attr.ConstructorArguments.Count > 0 ? attr.ConstructorArguments[0].Value : null;
        }
    }

    // Class to build assembly code
    public class AssemblyBuilder
    {
        private readonly string _architecture;
        private readonly List<string> _lines = new List<string>();

        public AssemblyBuilder(string architecture)
        {
            _architecture = architecture;

            // Add architecture-specific header
            switch (architecture)
            {
                case "x64":
                    AddLine(".code64");
                    break;
                case "x86":
                    AddLine(".code32");
                    break;
                case "arm64":
                    AddLine("// ARM64 Assembly");
                    break;
                case "arm32":
                    AddLine("// ARM32 Assembly");
                    break;
            }
        }

        public void AddComment(string comment)
        {
            AddLine($"; {comment}");
        }

        public void AddDirective(string directive)
        {
            AddLine(directive);
        }

        public void AddLine(string line)
        {
            _lines.Add(line);
        }

        public void AddMethodPrologue(MethodDef method)
        {
            AddLine($"{method.Name.Replace('.', '_')}:");

            switch (_architecture)
            {
                case "x64":
                    AddLine("    push rbp");
                    AddLine("    mov rbp, rsp");
                    AddLine("    sub rsp, 32");  // Basic stack frame
                    break;
                case "x86":
                    AddLine("    push ebp");
                    AddLine("    mov ebp, esp");
                    break;
                case "arm64":
                    AddLine("    stp x29, x30, [sp, #-16]!");
                    AddLine("    mov x29, sp");
                    break;
                case "arm32":
                    AddLine("    push {r7, lr}");
                    AddLine("    mov r7, sp");
                    break;
            }
        }

        public void AddMethodEpilogue(MethodDef method)
        {
            switch (_architecture)
            {
                case "x64":
                    AddLine("    mov rsp, rbp");
                    AddLine("    pop rbp");
                    AddLine("    ret");
                    break;
                case "x86":
                    AddLine("    mov esp, ebp");
                    AddLine("    pop ebp");
                    AddLine("    ret");
                    break;
                case "arm64":
                    AddLine("    ldp x29, x30, [sp], #16");
                    AddLine("    ret");
                    break;
                case "arm32":
                    AddLine("    pop {r7, pc}");
                    break;
            }
            AddLine("");  // Empty line between methods
        }

        public void ConvertILInstruction(Instruction instruction)
        {
            // Add a comment showing the original IL instruction
            AddComment($"IL_{instruction.Offset:X4}: {instruction.OpCode.Name} {(instruction.Operand != null ? instruction.Operand.ToString() : "")}");

            // This would be a complex mapping of IL opcodes to architecture-specific assembly
            // Here's a simplified example for a few common instructions
            switch (instruction.OpCode.Code)
            {
                case Code.UNKNOWN1:
                    break;
                case Code.UNKNOWN2:
                    break;
                // Arithmetic operations
                case Code.Add:
                    if (_architecture == "x64")
                        AddLine("    add rax, rbx");
                    else if (_architecture == "x86")
                        AddLine("    add eax, ebx");
                    else if (_architecture == "arm64")
                        AddLine("    add x0, x0, x1");
                    else if (_architecture == "arm32")
                        AddLine("    add r0, r0, r1");
                    break;
                case Code.Add_Ovf:
                    if (_architecture == "x64") {
                        AddLine("    add rax, rbx");
                        AddLine("    jo overflow_exception");  // Jump if overflow
                    } else if (_architecture == "x86") {
                        AddLine("    add eax, ebx");
                        AddLine("    jo overflow_exception");  // Jump if overflow
                    } else if (_architecture == "arm64") {
                        AddLine("    adds x0, x0, x1");
                        AddLine("    b.vs overflow_exception");  // Branch if overflow (V set)
                    } else if (_architecture == "arm32") {
                        AddLine("    adds r0, r0, r1");
                        AddLine("    bvs overflow_exception");  // Branch if overflow (V set)
                    }
                    break;
                case Code.Add_Ovf_Un:
                    if (_architecture == "x64") {
                        AddLine("    add rax, rbx");
                        AddLine("    jc overflow_exception");  // Jump if carry (unsigned overflow)
                    } else if (_architecture == "x86") {
                        AddLine("    add eax, ebx");
                        AddLine("    jc overflow_exception");  // Jump if carry (unsigned overflow)
                    } else if (_architecture == "arm64") {
                        AddLine("    adds x0, x0, x1");
                        AddLine("    b.cs overflow_exception");  // Branch if carry set (unsigned overflow)
                    } else if (_architecture == "arm32") {
                        AddLine("    adds r0, r0, r1");
                        AddLine("    bcs overflow_exception");  // Branch if carry set (unsigned overflow)
                    }
                    break;
                case Code.And:
                    if (_architecture == "x64")
                        AddLine("    and rax, rbx");
                    else if (_architecture == "x86")
                        AddLine("    and eax, ebx");
                    else if (_architecture == "arm64")
                        AddLine("    and x0, x0, x1");
                    else if (_architecture == "arm32")
                        AddLine("    and r0, r0, r1");
                    break;
                case Code.Arglist:
                    break;
                case Code.Beq:
                case Code.Beq_S:
                    if (instruction.Operand is Instruction eqInstr)
                    {
                        string label = $"IL_{eqInstr.Offset:X4}";
                        if (_architecture == "x64" || _architecture == "x86")
                        {
                            AddLine("    cmp rax, rbx");
                            AddLine($"    je {label}");
                        }
                        else if (_architecture == "arm64")
                        {
                            AddLine("    cmp x0, x1");
                            AddLine($"    b.eq {label}");
                        }
                        else if (_architecture == "arm32")
                        {
                            AddLine("    cmp r0, r1");
                            AddLine($"    beq {label}");
                        }
                    }
                    break;
                case Code.Bge:
                    break;
                case Code.Bge_S:
                    break;
                case Code.Bge_Un:
                    break;
                case Code.Bge_Un_S:
                    break;
                case Code.Bgt:
                    break;
                case Code.Bgt_S:
                    break;
                case Code.Bgt_Un_S:
                    break;
                case Code.Ble:
                    break;
                case Code.Blt:
                case Code.Blt_S:
                    if (instruction.Operand is Instruction ltInstr)
                    {
                        string label = $"IL_{ltInstr.Offset:X4}";
                        if (_architecture == "x64" || _architecture == "x86")
                        {
                            AddLine("    cmp rax, rbx");
                            AddLine($"    jl {label}");
                        }
                        else if (_architecture == "arm64")
                        {
                            AddLine("    cmp x0, x1");
                            AddLine($"    b.lt {label}");
                        }
                        else if (_architecture == "arm32")
                        {
                            AddLine("    cmp r0, r1");
                            AddLine($"    blt {label}");
                        }
                    }
                    break;
                case Code.Blt_Un:
                    break;
                case Code.Blt_Un_S:
                    break;
                case Code.Bne_Un:
                    break;
                case Code.Bne_Un_S:
                    break;
                case Code.Box:
                    if (instruction.Operand is ITypeDefOrRef boxType)
                    {
                        if (_architecture == "x64")
                            AddLine($"    call box_{boxType.Name.Replace('.', '_')}");
                        else if (_architecture == "x86")
                            AddLine($"    call box_{boxType.Name.Replace('.', '_')}");
                        else if (_architecture == "arm64")
                            AddLine($"    bl box_{boxType.Name.Replace('.', '_')}");
                        else if (_architecture == "arm32")
                            AddLine($"    bl box_{boxType.Name.Replace('.', '_')}");
                    }
                    break;
                case Code.Br:
                case Code.Br_S:
                    if (instruction.Operand is Instruction targetInstr)
                    {
                        string label = $"IL_{targetInstr.Offset:X4}";
                        if (_architecture == "x64" || _architecture == "x86")
                            AddLine($"    jmp {label}");
                        else if (_architecture == "arm64" || _architecture == "arm32")
                            AddLine($"    b {label}");
                    }
                    break;
                case Code.Break:
                    break;
                case Code.Brfalse:
                    break;
                case Code.Brfalse_S:
                    break;
                case Code.Brtrue:
                case Code.Brtrue_S:
                    if (instruction.Operand is Instruction trueInstr)
                    {
                        string label = $"IL_{trueInstr.Offset:X4}";
                        if (_architecture == "x64" || _architecture == "x86")
                        {
                            AddLine("    test rax, rax");
                            AddLine($"    jnz {label}");
                        }
                        else if (_architecture == "arm64")
                        {
                            AddLine("    cbnz x0, {label}");
                        }
                        else if (_architecture == "arm32")
                        {
                            AddLine("    cmp r0, #0");
                            AddLine($"    bne {label}");
                        }
                    }
                    break;
                case Code.Call:
                    if (instruction.Operand is IMethodDefOrRef methodRef)
                    {
                        string methodName = methodRef.Name.Replace('.', '_');
                        if (_architecture == "x64")
                            AddLine($"    call {methodName}");
                        else if (_architecture == "x86")
                            AddLine($"    call {methodName}");
                    }
                    break;
                case Code.Calli:
                    break;
                // Virtual method calls
                case Code.Callvirt:
                    if (instruction.Operand is IMethodDefOrRef virtMethodRef)
                    {
                        string methodName = virtMethodRef.Name.Replace('.', '_');
                        if (_architecture == "x64")
                        {
                            AddLine("    mov rax, [rcx]");  // Load vtable
                            int vtableOffset = 0; // Would need to calculate the correct vtable offset
                            AddLine($"    call [rax+{vtableOffset}]");  // Call through vtable
                        }
                        else if (_architecture == "x86")
                        {
                            AddLine("    mov eax, [eax]");  // Load vtable
                            int vtableOffset = 0; // Would need to calculate the correct vtable offset
                            AddLine($"    call [eax+{vtableOffset}]");
                        }
                        else if (_architecture == "arm64")
                        {
                            AddLine("    ldr x9, [x0]");  // Load vtable
                            int vtableOffset = 0;
                            AddLine($"    ldr x9, [x9, #{vtableOffset}]");
                            AddLine("    blr x9");
                        }
                        else if (_architecture == "arm32")
                        {
                            AddLine("    ldr r9, [r0]");  // Load vtable
                            int vtableOffset = 0;
                            AddLine($"    ldr r9, [r9, #{vtableOffset}]");
                            AddLine("    blx r9");
                        }
                    }
                    break;
                case Code.Castclass:
                    break;
                case Code.Ceq:
                    if (_architecture == "x64")
                    {
                        AddLine("    cmp rax, rbx");
                        AddLine("    sete al");
                        AddLine("    movzx rax, al");
                    }
                    else if (_architecture == "x86")
                    {
                        AddLine("    cmp eax, ebx");
                        AddLine("    sete al");
                        AddLine("    movzx eax, al");
                    }
                    else if (_architecture == "arm64")
                    {
                        AddLine("    cmp x0, x1");
                        AddLine("    cset x0, eq");
                    }
                    else if (_architecture == "arm32")
                    {
                        AddLine("    cmp r0, r1");
                        AddLine("    moveq r0, #1");
                        AddLine("    movne r0, #0");
                    }
                    break;
                case Code.Cgt:
                    break;
                case Code.Cgt_Un:
                    break;
                case Code.Ckfinite:
                    break;
                case Code.Clt:
                    if (_architecture == "x64")
                    {
                        AddLine("    cmp rax, rbx");
                        AddLine("    setl al");
                        AddLine("    movzx rax, al");
                    }
                    else if (_architecture == "x86")
                    {
                        AddLine("    cmp eax, ebx");
                        AddLine("    setl al");
                        AddLine("    movzx eax, al");
                    }
                    else if (_architecture == "arm64")
                    {
                        AddLine("    cmp x0, x1");
                        AddLine("    cset x0, lt");
                    }
                    else if (_architecture == "arm32")
                    {
                        AddLine("    cmp r0, r1");
                        AddLine("    movlt r0, #1");
                        AddLine("    movge r0, #0");
                    }
                    break;
                case Code.Clt_Un:
                    break;
                case Code.Constrained:
                    break;
                case Code.Conv_I:
                    break;
                case Code.Conv_I1:
                    break;
                case Code.Conv_I2:
                    break;
                case Code.Conv_I4:
                    if (_architecture == "x64")
                        AddLine("    movsxd rax, eax");  // Sign extend 32-bit to 64-bit
                    else if (_architecture == "arm64")
                        AddLine("    sxtw x0, w0");
                    // x86 and arm32 don't need conversion for int32
                    break;
                case Code.Conv_I8:
                    break;
                case Code.Conv_Ovf_I:
                    break;
                case Code.Conv_Ovf_I_Un:
                    break;
                case Code.Conv_Ovf_I1:
                    break;
                case Code.Conv_Ovf_I1_Un:
                    break;
                case Code.Conv_Ovf_I2:
                    break;
                case Code.Conv_Ovf_I2_Un:
                    break;
                case Code.Conv_Ovf_I4:
                    break;
                case Code.Conv_Ovf_I4_Un:
                    break;
                case Code.Conv_Ovf_I8:
                    break;
                case Code.Conv_Ovf_I8_Un:
                    break;
                case Code.Conv_Ovf_U:
                    break;
                case Code.Conv_Ovf_U_Un:
                    break;
                case Code.Conv_Ovf_U1:
                    break;
                case Code.Conv_Ovf_U1_Un:
                    break;
                case Code.Conv_Ovf_U2:
                    break;
                case Code.Conv_Ovf_U2_Un:
                    break;
                case Code.Conv_Ovf_U4:
                    break;
                case Code.Conv_Ovf_U4_Un:
                    break;
                case Code.Conv_Ovf_U8:
                    break;
                case Code.Conv_Ovf_U8_Un:
                    break;
                case Code.Conv_R_Un:
                    break;
                case Code.Conv_R4:
                    break;
                case Code.Conv_R8:
                    break;
                case Code.Conv_U:
                    break;
                case Code.Conv_U1:
                    break;
                case Code.Conv_U2:
                    break;
                case Code.Conv_U4:
                    break;
                case Code.Conv_U8:
                    break;
                case Code.Cpblk:
                    break;
                case Code.Cpobj:
                    break;
                case Code.Div:
                    if (_architecture == "x64")
                    {
                        AddLine("    xor rdx, rdx");
                        AddLine("    div rbx");
                    }
                    else if (_architecture == "x86")
                    {
                        AddLine("    xor edx, edx");
                        AddLine("    div ebx");
                    }
                    else if (_architecture == "arm64")
                        AddLine("    udiv x0, x0, x1");
                    else if (_architecture == "arm32")
                        AddLine("    udiv r0, r0, r1");
                    break;
                case Code.Div_Un:
                    break;
                case Code.Dup:
                    if (_architecture == "x64")
                        AddLine("    push rax");  // Duplicate top of stack
                    else if (_architecture == "x86")
                        AddLine("    push eax");
                    else if (_architecture == "arm64")
                        AddLine("    mov x1, x0");  // ARM doesn't have direct stack duplicate
                    else if (_architecture == "arm32")
                        AddLine("    mov r1, r0");
                    break;
                case Code.Endfilter:
                    break;
                case Code.Endfinally:
                    break;
                case Code.Initblk:
                    break;
                case Code.Initobj:
                    break;
                case Code.Isinst:
                    break;
                case Code.Jmp:
                    break;
                case Code.Ldarg:
                    break;
                case Code.Ldarg_0:
                    if (_architecture == "x64")
                        AddLine("    mov rax, rcx");  // First argument in x64 calling convention
                    else if (_architecture == "x86")
                        AddLine("    mov eax, [ebp+8]");
                    break;
                case Code.Ldarg_1:
                    if (_architecture == "x64")
                        AddLine("    mov rax, rdx");  // Second argument in x64 calling convention
                    else if (_architecture == "x86")
                        AddLine("    mov eax, [ebp+12]");
                    else if (_architecture == "arm64")
                        AddLine("    mov x0, x1");  // Second argument
                    else if (_architecture == "arm32")
                        AddLine("    mov r0, r1");  // Second argument
                    break;

                case Code.Ldarg_2:
                    if (_architecture == "x64")
                        AddLine("    mov rax, r8");  // Third argument in x64 calling convention
                    else if (_architecture == "x86")
                        AddLine("    mov eax, [ebp+16]");
                    else if (_architecture == "arm64")
                        AddLine("    mov x0, x2");  // Third argument
                    else if (_architecture == "arm32")
                        AddLine("    mov r0, r2");  // Third argument
                    break;
                case Code.Ldarg_3:
                    break;
                case Code.Ldarg_S:
                    break;
                case Code.Ldarga:
                    break;
                case Code.Ldarga_S:
                    break;
                case Code.Ldc_I4:
                    if (_architecture == "x64")
                        AddLine($"    mov eax, {instruction.Operand}");
                    else if (_architecture == "x86")
                        AddLine($"    mov eax, {instruction.Operand}");
                    break;
                case Code.Ldc_I4_0:
                    break;
                case Code.Ldc_I4_1:
                    break;
                case Code.Ldc_I4_2:
                    break;
                case Code.Ldc_I4_3:
                    break;
                case Code.Ldc_I4_4:
                    break;
                case Code.Ldc_I4_5:
                    break;
                case Code.Ldc_I4_6:
                    break;
                case Code.Ldc_I4_7:
                    break;
                case Code.Ldc_I4_8:
                    break;
                case Code.Ldc_I4_M1:
                    break;
                case Code.Ldc_I4_S:
                    break;
                case Code.Ldc_I8:
                    break;
                case Code.Ldc_R4:
                    break;
                case Code.Ldc_R8:
                    break;
                case Code.Ldelem:
                    break;
                case Code.Ldelem_I:
                    break;
                case Code.Ldelem_I1:
                    break;
                case Code.Ldelem_I2:
                    break;
                case Code.Ldelem_I4:
                    break;
                case Code.Ldelem_I8:
                    break;
                case Code.Ldelem_R4:
                    break;
                case Code.Ldelem_R8:
                    break;
                case Code.Ldelem_Ref:
                    break;
                case Code.Ldelem_U1:
                    break;
                case Code.Ldelem_U2:
                    break;
                case Code.Ldelem_U4:
                    break;
                case Code.Ldelema:
                    break;
                case Code.Ldfld:
                    if (instruction.Operand is IField field)
                    {
                        string fieldName = field.Name.Replace('.', '_');
                        int offset = 8; // This would be calculated based on field offset
                        if (_architecture == "x64")
                            AddLine($"    mov rax, [rcx+{offset}]");  // Assuming object in rcx
                        else if (_architecture == "x86")
                            AddLine($"    mov eax, [eax+{offset}]");  // Assuming object in eax
                        else if (_architecture == "arm64")
                            AddLine($"    ldr x0, [x0, #{offset}]");
                        else if (_architecture == "arm32")
                            AddLine($"    ldr r0, [r0, #{offset}]");
                    }
                    break;

                case Code.Ldflda:
                    break;
                case Code.Ldftn:
                    break;
                case Code.Ldind_I:
                    break;
                case Code.Ldind_I1:
                    break;
                case Code.Ldind_I2:
                    break;
                case Code.Ldind_I4:
                    break;
                case Code.Ldind_I8:
                    break;
                case Code.Ldind_R4:
                    break;
                case Code.Ldind_R8:
                    break;
                case Code.Ldind_Ref:
                    break;
                case Code.Ldind_U1:
                    break;
                case Code.Ldind_U2:
                    break;
                case Code.Ldind_U4:
                    break;
                case Code.Ldlen:
                    if (_architecture == "x64")
                        AddLine("    mov rax, [rax-8]");  // Array length is typically stored before the array data
                    else if (_architecture == "x86")
                        AddLine("    mov eax, [eax-4]");
                    else if (_architecture == "arm64")
                        AddLine("    ldr x0, [x0, #-8]");
                    else if (_architecture == "arm32")
                        AddLine("    ldr r0, [r0, #-4]");
                    break;
                case Code.Ldloc:
                    break;
                case Code.Ldloc_0:
                    if (_architecture == "x64")
                        AddLine("    mov rax, [rbp-8]");
                    else if (_architecture == "x86")
                        AddLine("    mov eax, [ebp-4]");
                    else if (_architecture == "arm64")
                        AddLine("    ldr x0, [x29, #-8]");
                    else if (_architecture == "arm32")
                        AddLine("    ldr r0, [r7, #-4]");
                    break;
                case Code.Ldloc_1:
                    if (_architecture == "x64")
                        AddLine("    mov rax, [rbp-16]");
                    else if (_architecture == "x86")
                        AddLine("    mov eax, [ebp-8]");
                    else if (_architecture == "arm64")
                        AddLine("    ldr x0, [x29, #-16]");
                    else if (_architecture == "arm32")
                        AddLine("    ldr r0, [r7, #-8]");
                    break; ;
                case Code.Ldloc_2:
                    break;
                case Code.Ldloc_3:
                    break;
                case Code.Ldloc_S:
                    break;
                case Code.Ldloca:
                    break;
                case Code.Ldloca_S:
                    break;
                case Code.Ldnull:
                    break;
                case Code.Ldobj:
                    break;
                case Code.Ldsfld:
                    break;
                case Code.Ldsflda:
                    break;
                case Code.Ldstr:
                    // String literals would need special handling in the data section
                    string strValue = instruction.Operand as string;
                    if (strValue != null)
                    {
                        string strLabel = $"str_{Math.Abs(strValue.GetHashCode()):X8}";
                        if (_architecture == "x64")
                            AddLine($"    lea rax, [{strLabel}]");
                        else if (_architecture == "x86")
                            AddLine($"    lea eax, [{strLabel}]");
                        else if (_architecture == "arm64")
                            AddLine($"    adr x0, {strLabel}");
                        else if (_architecture == "arm32")
                            AddLine($"    adr r0, {strLabel}");
                    }
                    break;
                case Code.Ldtoken:
                    break;
                case Code.Ldvirtftn:
                    break;
                case Code.Leave:
                    break;
                case Code.Leave_S:
                    break;
                case Code.Localloc:
                    break;
                case Code.Mkrefany:
                    break;
                case Code.Mul:
                    if (_architecture == "x64")
                        AddLine("    imul rax, rbx");
                    else if (_architecture == "x86")
                        AddLine("    imul eax, ebx");
                    else if (_architecture == "arm64")
                        AddLine("    mul x0, x0, x1");
                    else if (_architecture == "arm32")
                        AddLine("    mul r0, r0, r1");
                    break;
                case Code.Mul_Ovf:
                    if (_architecture == "x64")
                    {
                        AddLine("    imul rax, rbx");
                        AddLine("    jo overflow_exception");  // Jump if overflow
                    }
                    else if (_architecture == "x86")
                    {
                        AddLine("    imul eax, ebx");
                        AddLine("    jo overflow_exception");  // Jump if overflow
                    }
                    else if (_architecture == "arm64")
                    {
                        AddLine("    smull x0, w0, w1");  // Signed multiply with potential overflow detection
                        // Check if upper 32 bits are all 0 or all 1 (sign extension)
                        AddLine("    asr x1, x0, #63");  // Arithmetic shift right to get sign bits
                        AddLine("    cmp x1, #0");       // Should be all 0s for positive number
                        AddLine("    ccmp x1, #-1, #0, ne");  // Should be all 1s for negative number
                        AddLine("    b.ne overflow_exception");  // If neither, overflow
                    }
                    else if (_architecture == "arm32")
                    {
                        AddLine("    smull r0, r3, r0, r1");  // Result in r0 (low) and r3 (high)
                        // Check if high register matches sign-extension of low register
                        AddLine("    asr r2, r0, #31");  // Get sign of lower word
                        AddLine("    cmp r2, r3");       // Compare with high word
                        AddLine("    bne overflow_exception");  // If different, overflow
                    }
                    break;
                case Code.Mul_Ovf_Un:
                    if (_architecture == "x64")
                    {
                        AddLine("    mul rbx");          // Unsigned multiply, RDX:RAX = RAX * RBX
                        AddLine("    test rdx, rdx");    // Check if high bits are zero
                        AddLine("    jnz overflow_exception");  // If not zero, overflow
                    }
                    else if (_architecture == "x86")
                    {
                        AddLine("    mul ebx");          // Unsigned multiply, EDX:EAX = EAX * EBX
                        AddLine("    test edx, edx");    // Check if high bits are zero
                        AddLine("    jnz overflow_exception");  // If not zero, overflow
                    }
                    else if (_architecture == "arm64")
                    {
                        AddLine("    mul x0, x0, x1");   // Multiply
                        AddLine("    umulh x1, x0, x1"); // Get high 64 bits of product
                        AddLine("    cbnz x1, overflow_exception");  // If high bits not zero, overflow
                    }
                    else if (_architecture == "arm32")
                    {
                        AddLine("    umull r0, r3, r0, r1");  // Result in r0 (low) and r3 (high)
                        AddLine("    cmp r3, #0");           // Check if high bits are zero
                        AddLine("    bne overflow_exception");  // If not zero, overflow
                    }
                    break;
                case Code.Neg:
                    if (_architecture == "x64")
                        AddLine("    neg rax");  // Negate value in RAX
                    else if (_architecture == "x86")
                        AddLine("    neg eax");  // Negate value in EAX
                    else if (_architecture == "arm64")
                        AddLine("    neg x0, x0");  // Negate value in X0
                    else if (_architecture == "arm32")
                        AddLine("    neg r0, r0");  // Negate value in R0
                    break;
                case Code.Newarr:
                    if (instruction.Operand is ITypeDefOrRef arrType)
                    {
                        if (_architecture == "x64")
                        {
                            AddLine("    mov rcx, rax");  // Size already in rax
                            AddLine($"    call new_array_{arrType.Name.Replace('.', '_')}");
                        }
                        else if (_architecture == "x86")
                        {
                            AddLine("    push eax");  // Size already in eax
                            AddLine($"    call new_array_{arrType.Name.Replace('.', '_')}");
                        }
                        else if (_architecture == "arm64")
                        {
                            AddLine($"    bl new_array_{arrType.Name.Replace('.', '_')}");
                        }
                        else if (_architecture == "arm32")
                        {
                            AddLine($"    bl new_array_{arrType.Name.Replace('.', '_')}");
                        }
                    }
                    break;
                case Code.Newobj:
                    if (instruction.Operand is IMethodDefOrRef ctorRef)
                    {
                        string typeName = ctorRef.DeclaringType.Name.Replace('.', '_');
                        if (_architecture == "x64")
                        {
                            // Simplified object creation - would need runtime support
                            AddLine($"    call alloc_{typeName}");
                            AddLine($"    call {ctorRef.Name.Replace('.', '_')}");
                        }
                        else if (_architecture == "x86")
                        {
                            AddLine($"    call alloc_{typeName}");
                            AddLine($"    call {ctorRef.Name.Replace('.', '_')}");
                        }
                        else if (_architecture == "arm64")
                        {
                            AddLine($"    bl alloc_{typeName}");
                            AddLine($"    bl {ctorRef.Name.Replace('.', '_')}");
                        }
                        else if (_architecture == "arm32")
                        {
                            AddLine($"    bl alloc_{typeName}");
                            AddLine($"    bl {ctorRef.Name.Replace('.', '_')}");
                        }
                    }
                    break;
                case Code.No:
                    // No operation or placeholder - typically doesn't generate any code
                    AddLine("    ; Invalid or unrecognized opcode");
                    break;
                case Code.Nop:
                    // No operation - used for padding or alignment
                    if (_architecture == "x64" || _architecture == "x86")
                        AddLine("    nop");
                    else if (_architecture == "arm64")
                        AddLine("    nop");
                    else if (_architecture == "arm32")
                        AddLine("    nop");
                    break;
                case Code.Not:
                    // Bitwise NOT operation
                    if (_architecture == "x64")
                        AddLine("    not rax");
                    else if (_architecture == "x86")
                        AddLine("    not eax");
                    else if (_architecture == "arm64")
                        AddLine("    mvn x0, x0");  // Bitwise NOT of x0
                    else if (_architecture == "arm32")
                        AddLine("    mvn r0, r0");  // Bitwise NOT of r0
                    break;
                case Code.Or:
                    if (_architecture == "x64")
                        AddLine("    or rax, rbx");
                    else if (_architecture == "x86")
                        AddLine("    or eax, ebx");
                    else if (_architecture == "arm64")
                        AddLine("    orr x0, x0, x1");
                    else if (_architecture == "arm32")
                        AddLine("    orr r0, r0, r1");
                    break;
                case Code.Pop:
                    if (_architecture == "x64")
                        AddLine("    pop rax");  // Discard top of stack
                    else if (_architecture == "x86")
                        AddLine("    pop eax");
                    else if (_architecture == "arm64" || _architecture == "arm32")
                        AddLine("    ; pop implemented by register reassignment");
                    break;
                case Code.Prefix1:
                    break;
                case Code.Prefix2:
                    break;
                case Code.Prefix3:
                    break;
                case Code.Prefix4:
                    break;
                case Code.Prefix5:
                    break;
                case Code.Prefix6:
                    break;
                case Code.Prefix7:
                    break;
                case Code.Prefixref:
                    break;
                case Code.Readonly:
                    break;
                case Code.Refanytype:
                    break;
                case Code.Refanyval:
                    break;
                case Code.Rem:
                    break;
                case Code.Rem_Un:
                    break;
                case Code.Ret:
                    // Return instruction is handled by method epilogue
                    // No additional assembly needed here
                    break;
                case Code.Rethrow:
                    break;
                case Code.Shl:
                    if (_architecture == "x64")
                        AddLine("    shl rax, cl");  // Shift count in CL register
                    else if (_architecture == "x86")
                        AddLine("    shl eax, cl");
                    else if (_architecture == "arm64")
                        AddLine("    lsl x0, x0, x1");
                    else if (_architecture == "arm32")
                        AddLine("    lsl r0, r0, r1");
                    break;
                case Code.Shr:
                    if (_architecture == "x64")
                        AddLine("    shr rax, cl");
                    else if (_architecture == "x86")
                        AddLine("    shr eax, cl");
                    else if (_architecture == "arm64")
                        AddLine("    lsr x0, x0, x1");
                    else if (_architecture == "arm32")
                        AddLine("    lsr r0, r0, r1");
                    break;
                case Code.Shr_Un:
                    if (_architecture == "x64")
                        AddLine("    shr rax, cl");  // Unsigned right shift, high bits filled with zeros
                    else if (_architecture == "x86")
                        AddLine("    shr eax, cl");  // Unsigned right shift for x86
                    else if (_architecture == "arm64")
                        AddLine("    lsr x0, x0, x1");  // Logical shift right (unsigned)
                    else if (_architecture == "arm32")
                        AddLine("    lsr r0, r0, r1");  // Logical shift right for ARM32
                    break;
                case Code.Sizeof:
                    if (instruction.Operand is ITypeDefOrRef typeDef)
                    {
                        // Calculate size based on type - simplified implementation
                        int size = 4;  // Default size for most value types
                        if (typeDef.Name == "Byte" || typeDef.Name == "SByte")
                            size = 1;
                        else if (typeDef.Name == "Int16" || typeDef.Name == "UInt16" || typeDef.Name == "Char")
                            size = 2;
                        else if (typeDef.Name == "Int64" || typeDef.Name == "UInt64" || typeDef.Name == "Double")
                            size = 8;

                        if (_architecture == "x64")
                            AddLine($"    mov eax, {size}");  // Load size into return register
                        else if (_architecture == "x86")
                            AddLine($"    mov eax, {size}");
                        else if (_architecture == "arm64")
                            AddLine($"    mov x0, #{size}");
                        else if (_architecture == "arm32")
                            AddLine($"    mov r0, #{size}");
                    }
                    break;
                case Code.Starg:
                    if (instruction.Operand is Parameter param)
                    {
                        int index = param.Index;
                        int offset = (index + 1) * 8;  // Calculate stack position

                        if (_architecture == "x64")
                        {
                            // In x64, first 4 args are in registers, rest on stack
                            if (index == 0)
                                AddLine("    mov rcx, rax");  // First argument
                            else if (index == 1)
                                AddLine("    mov rdx, rax");  // Second argument
                            else if (index == 2)
                                AddLine("    mov r8, rax");   // Third argument
                            else if (index == 3)
                                AddLine("    mov r9, rax");   // Fourth argument
                            else
                                AddLine($"    mov [rbp+{offset + 16}], rax");  // Stack-based argument
                        }
                        else if (_architecture == "x86")
                        {
                            // In x86, all arguments are on stack
                            AddLine($"    mov [ebp+{offset + 4}], eax");
                        }
                        else if (_architecture == "arm64")
                        {
                            if (index < 8)
                                AddLine($"    mov x{index}, x0");  // First 8 args in registers
                            else
                                AddLine($"    str x0, [x29, #{offset + 16}]");  // Rest on stack
                        }
                        else if (_architecture == "arm32")
                        {
                            if (index < 4)
                                AddLine($"    mov r{index}, r0");  // First 4 args in registers
                            else
                                AddLine($"    str r0, [r7, #{offset + 4}]");  // Rest on stack
                        }
                    }
                    break;
                case Code.Starg_S:
                    if (instruction.Operand is Parameter paramS)
                    {
                        int index = paramS.Index;
                        int offset = (index + 1) * 8;  // Calculate stack position

                        // Implementation is the same as Starg, just with a smaller operand encoding
                        if (_architecture == "x64")
                        {
                            if (index == 0)
                                AddLine("    mov rcx, rax");
                            else if (index == 1)
                                AddLine("    mov rdx, rax");
                            else if (index == 2)
                                AddLine("    mov r8, rax");
                            else if (index == 3)
                                AddLine("    mov r9, rax");
                            else
                                AddLine($"    mov [rbp+{offset + 16}], rax");
                        }
                        else if (_architecture == "x86")
                        {
                            AddLine($"    mov [ebp+{offset + 4}], eax");
                        }
                        else if (_architecture == "arm64")
                        {
                            if (index < 8)
                                AddLine($"    mov x{index}, x0");
                            else
                                AddLine($"    str x0, [x29, #{offset + 16}]");
                        }
                        else if (_architecture == "arm32")
                        {
                            if (index < 4)
                                AddLine($"    mov r{index}, r0");
                            else
                                AddLine($"    str r0, [r7, #{offset + 4}]");
                        }
                    }
                    break;
                case Code.Stelem:
                    if (instruction.Operand is ITypeDefOrRef elemType)
                    {
                        int elemSize = 4; // Default size, would need to be calculated based on type
                        if (_architecture == "x64")
                        {
                            // Assuming: array in rcx, index in rdx, value in r8
                            AddLine("    mov rax, rdx");  // Copy index
                            AddLine($"    imul rax, {elemSize}");  // Multiply by element size
                            AddLine("    add rax, 16");  // Skip array header (length, etc.)
                            AddLine("    mov [rcx + rax], r8");  // Store value
                        }
                        else if (_architecture == "x86")
                        {
                            // Assuming array, index, and value are on stack
                            AddLine("    mov ecx, [esp+8]");  // Load array reference
                            AddLine("    mov edx, [esp+4]");  // Load index
                            AddLine("    mov eax, [esp]");    // Load value
                            AddLine($"    imul edx, {elemSize}");  // Multiply by element size
                            AddLine("    add edx, 12");  // Skip array header
                            AddLine("    mov [ecx + edx], eax");  // Store value
                        }
                        else if (_architecture == "arm64")
                        {
                            // Assuming: array in x0, index in x1, value in x2
                            AddLine($"    mov x9, #{elemSize}");  // Load element size
                            AddLine("    mul x9, x1, x9");  // Calculate offset = index * elemSize
                            AddLine("    add x9, x9, #16");  // Skip array header
                            AddLine("    str x2, [x0, x9]");  // Store value
                        }
                        else if (_architecture == "arm32")
                        {
                            // Assuming: array in r0, index in r1, value in r2
                            AddLine($"    mov r3, #{elemSize}");  // Load element size
                            AddLine("    mul r3, r1, r3");  // Calculate offset
                            AddLine("    add r3, r3, #12");  // Skip array header
                            AddLine("    str r2, [r0, r3]");  // Store value
                        }
                    }
                    break;
                case Code.Stelem_I:
                    if (_architecture == "x64")
                    {
                        // Assuming: array in rcx, index in rdx, value in r8
                        AddLine("    mov rax, rdx");  // Copy index
                        AddLine("    shl rax, 3");  // Multiply by 8 (sizeof(IntPtr) on x64)
                        AddLine("    add rax, 16");  // Skip array header
                        AddLine("    mov [rcx + rax], r8");  // Store native int
                    }
                    else if (_architecture == "x86")
                    {
                        AddLine("    mov ecx, [esp+8]");  // Load array reference
                        AddLine("    mov edx, [esp+4]");  // Load index
                        AddLine("    mov eax, [esp]");    // Load value
                        AddLine("    shl edx, 2");  // Multiply by 4 (sizeof(IntPtr) on x86)
                        AddLine("    add edx, 12");  // Skip array header
                        AddLine("    mov [ecx + edx], eax");  // Store native int
                    }
                    else if (_architecture == "arm64")
                    {
                        AddLine("    lsl x9, x1, #3");  // Multiply index by 8
                        AddLine("    add x9, x9, #16");  // Skip array header
                        AddLine("    str x2, [x0, x9]");  // Store native int
                    }
                    else if (_architecture == "arm32")
                    {
                        AddLine("    lsl r3, r1, #2");  // Multiply index by 4
                        AddLine("    add r3, r3, #12");  // Skip array header
                        AddLine("    str r2, [r0, r3]");  // Store native int
                    }
                    break;
                case Code.Stelem_I1:
                    if (_architecture == "x64")
                    {
                        // Store byte
                        AddLine("    add rcx, 16");  // Skip array header
                        AddLine("    add rcx, rdx");  // Add index (1 byte per element)
                        AddLine("    mov [rcx], r8b");  // Store byte
                    }
                    else if (_architecture == "x86")
                    {
                        AddLine("    mov ecx, [esp+8]");  // Array
                        AddLine("    mov edx, [esp+4]");  // Index
                        AddLine("    mov al, [esp]");     // Value (byte)
                        AddLine("    add ecx, 12");  // Skip array header
                        AddLine("    add ecx, edx");  // Add index
                        AddLine("    mov [ecx], al");  // Store byte
                    }
                    else if (_architecture == "arm64")
                    {
                        AddLine("    add x0, x0, #16");  // Skip array header
                        AddLine("    add x0, x0, x1");   // Add index
                        AddLine("    strb w2, [x0]");    // Store byte
                    }
                    else if (_architecture == "arm32")
                    {
                        AddLine("    add r0, r0, #12");  // Skip array header
                        AddLine("    add r0, r0, r1");   // Add index
                        AddLine("    strb r2, [r0]");    // Store byte
                    }
                    break;
                case Code.Stelem_I2:
                    if (_architecture == "x64")
                    {
                        // Store 16-bit short
                        AddLine("    mov rax, rdx");  // Copy index
                        AddLine("    shl rax, 1");    // Multiply by 2 (sizeof(short))
                        AddLine("    add rcx, 16");   // Skip array header
                        AddLine("    add rcx, rax");  // Add offset
                        AddLine("    mov [rcx], r8w"); // Store 16-bit value
                    }
                    else if (_architecture == "x86")
                    {
                        AddLine("    mov ecx, [esp+8]");  // Array
                        AddLine("    mov edx, [esp+4]");  // Index
                        AddLine("    mov ax, [esp]");     // Value (short)
                        AddLine("    shl edx, 1");        // Multiply by 2
                        AddLine("    add ecx, 12");       // Skip array header
                        AddLine("    add ecx, edx");      // Add offset
                        AddLine("    mov [ecx], ax");     // Store short
                    }
                    else if (_architecture == "arm64")
                    {
                        AddLine("    lsl x9, x1, #1");    // Multiply index by 2
                        AddLine("    add x0, x0, #16");   // Skip array header
                        AddLine("    add x0, x0, x9");    // Add offset
                        AddLine("    strh w2, [x0]");     // Store halfword
                    }
                    else if (_architecture == "arm32")
                    {
                        AddLine("    lsl r3, r1, #1");    // Multiply index by 2
                        AddLine("    add r0, r0, #12");   // Skip array header
                        AddLine("    add r0, r0, r3");    // Add offset
                        AddLine("    strh r2, [r0]");     // Store halfword
                    }
                    break;
                case Code.Stelem_I4:
                    if (_architecture == "x64")
                    {
                        // Store 32-bit int
                        AddLine("    mov rax, rdx");     // Copy index
                        AddLine("    shl rax, 2");       // Multiply by 4 (sizeof(int))
                        AddLine("    add rcx, 16");      // Skip array header
                        AddLine("    add rcx, rax");     // Add offset
                        AddLine("    mov [rcx], r8d");   // Store 32-bit value
                    }
                    else if (_architecture == "x86")
                    {
                        AddLine("    mov ecx, [esp+8]");  // Array
                        AddLine("    mov edx, [esp+4]");  // Index
                        AddLine("    mov eax, [esp]");    // Value (int)
                        AddLine("    shl edx, 2");        // Multiply by 4
                        AddLine("    add ecx, 12");       // Skip array header
                        AddLine("    add ecx, edx");      // Add offset
                        AddLine("    mov [ecx], eax");    // Store int
                    }
                    else if (_architecture == "arm64")
                    {
                        AddLine("    lsl x9, x1, #2");    // Multiply index by 4
                        AddLine("    add x0, x0, #16");   // Skip array header
                        AddLine("    add x0, x0, x9");    // Add offset
                        AddLine("    str w2, [x0]");      // Store word
                    }
                    else if (_architecture == "arm32")
                    {
                        AddLine("    lsl r3, r1, #2");    // Multiply index by 4
                        AddLine("    add r0, r0, #12");   // Skip array header
                        AddLine("    add r0, r0, r3");    // Add offset
                        AddLine("    str r2, [r0]");      // Store word
                    }
                    break;
                case Code.Stelem_I8:
                    if (_architecture == "x64")
                    {
                        // Store 64-bit long
                        AddLine("    mov rax, rdx");     // Copy index
                        AddLine("    shl rax, 3");       // Multiply by 8 (sizeof(long))
                        AddLine("    add rcx, 16");      // Skip array header
                        AddLine("    add rcx, rax");     // Add offset
                        AddLine("    mov [rcx], r8");    // Store 64-bit value
                    }
                    else if (_architecture == "x86")
                    {
                        // On x86, long is stored as two 32-bit values
                        AddLine("    mov ecx, [esp+12]");  // Array
                        AddLine("    mov edx, [esp+8]");   // Index
                        AddLine("    mov eax, [esp+4]");   // Low 32-bits of value
                        AddLine("    mov ebx, [esp]");     // High 32-bits of value
                        AddLine("    shl edx, 3");         // Multiply by 8
                        AddLine("    add ecx, 12");        // Skip array header
                        AddLine("    add ecx, edx");       // Add offset
                        AddLine("    mov [ecx], eax");     // Store low 32-bits
                        AddLine("    mov [ecx+4], ebx");   // Store high 32-bits (assuming in EDX)
                    }
                    else if (_architecture == "arm64")
                    {
                        AddLine("    lsl x9, x1, #3");     // Multiply index by 8
                        AddLine("    add x0, x0, #16");    // Skip array header
                        AddLine("    add x0, x0, x9");     // Add offset
                        AddLine("    str x2, [x0]");       // Store doubleword
                    }
                    else if (_architecture == "arm32")
                    {
                        // On ARM32, long is stored as two 32-bit values
                        AddLine("    lsl r3, r1, #3");     // Multiply index by 8
                        AddLine("    add r0, r0, #12");    // Skip array header
                        AddLine("    add r0, r0, r3");     // Add offset
                        AddLine("    str r2, [r0]");       // Store low 32-bits
                        AddLine("    str r3, [r0, #4]");   // Store high 32-bits (assumed in r3)
                    }
                    break;
                case Code.Stelem_R4:
                    if (_architecture == "x64")
                    {
                        // Store single-precision float
                        AddLine("    mov rax, rdx");       // Copy index
                        AddLine("    shl rax, 2");         // Multiply by 4 (sizeof(float))
                        AddLine("    add rcx, 16");        // Skip array header
                        AddLine("    add rcx, rax");       // Add offset
                        AddLine("    movss dword ptr [rcx], xmm2");  // Store float
                    }
                    else if (_architecture == "x86")
                    {
                        AddLine("    mov ecx, [esp+8]");   // Array
                        AddLine("    mov edx, [esp+4]");   // Index
                        AddLine("    movss xmm0, [esp]");  // Value (float)
                        AddLine("    shl edx, 2");         // Multiply by 4
                        AddLine("    add ecx, 12");        // Skip array header
                        AddLine("    add ecx, edx");       // Add offset
                        AddLine("    movss [ecx], xmm0");  // Store float
                    }
                    else if (_architecture == "arm64")
                    {
                        AddLine("    lsl x9, x1, #2");     // Multiply index by 4
                        AddLine("    add x0, x0, #16");    // Skip array header
                        AddLine("    add x0, x0, x9");     // Add offset
                        AddLine("    str s0, [x0]");       // Store float
                    }
                    else if (_architecture == "arm32")
                    {
                        AddLine("    lsl r3, r1, #2");     // Multiply index by 4
                        AddLine("    add r0, r0, #12");    // Skip array header
                        AddLine("    add r0, r0, r3");     // Add offset
                        AddLine("    vstr s0, [r0]");      // Store float
                    }
                    break;
                case Code.Stelem_R8:
                    if (_architecture == "x64")
                    {
                        // Store double-precision float
                        AddLine("    mov rax, rdx");       // Copy index
                        AddLine("    shl rax, 3");         // Multiply by 8 (sizeof(double))
                        AddLine("    add rcx, 16");        // Skip array header
                        AddLine("    add rcx, rax");       // Add offset
                        AddLine("    movsd qword ptr [rcx], xmm2");  // Store double
                    }
                    else if (_architecture == "x86")
                    {
                        AddLine("    mov ecx, [esp+8]");   // Array
                        AddLine("    mov edx, [esp+4]");   // Index
                        AddLine("    movsd xmm0, [esp]");  // Value (double)
                        AddLine("    shl edx, 3");         // Multiply by 8
                        AddLine("    add ecx, 12");        // Skip array header
                        AddLine("    add ecx, edx");       // Add offset
                        AddLine("    movsd [ecx], xmm0");  // Store double
                    }
                    else if (_architecture == "arm64")
                    {
                        AddLine("    lsl x9, x1, #3");     // Multiply index by 8
                        AddLine("    add x0, x0, #16");    // Skip array header
                        AddLine("    add x0, x0, x9");     // Add offset
                        AddLine("    str d0, [x0]");       // Store double
                    }
                    else if (_architecture == "arm32")
                    {
                        AddLine("    lsl r3, r1, #3");     // Multiply index by 8
                        AddLine("    add r0, r0, #12");    // Skip array header
                        AddLine("    add r0, r0, r3");     // Add offset
                        AddLine("    vstr d0, [r0]");      // Store double
                    }
                    break;
                case Code.Stelem_Ref:
                    if (_architecture == "x64")
                    {
                        // Store object reference
                        AddLine("    mov rax, rdx");      // Copy index
                        AddLine("    shl rax, 3");        // Multiply by 8 (sizeof(reference) on x64)
                        AddLine("    add rcx, 16");       // Skip array header
                        AddLine("    add rcx, rax");      // Add offset
                        AddLine("    mov [rcx], r8");     // Store reference
                    }
                    else if (_architecture == "x86")
                    {
                        AddLine("    mov ecx, [esp+8]");  // Array
                        AddLine("    mov edx, [esp+4]");  // Index
                        AddLine("    mov eax, [esp]");    // Value (reference)
                        AddLine("    shl edx, 2");        // Multiply by 4 (sizeof(reference) on x86)
                        AddLine("    add ecx, 12");       // Skip array header
                        AddLine("    add ecx, edx");      // Add offset
                        AddLine("    mov [ecx], eax");    // Store reference
                    }
                    else if (_architecture == "arm64")
                    {
                        AddLine("    lsl x9, x1, #3");    // Multiply index by 8
                        AddLine("    add x0, x0, #16");   // Skip array header
                        AddLine("    add x0, x0, x9");    // Add offset
                        AddLine("    str x2, [x0]");      // Store reference
                    }
                    else if (_architecture == "arm32")
                    {
                        AddLine("    lsl r3, r1, #2");    // Multiply index by 4
                        AddLine("    add r0, r0, #12");   // Skip array header
                        AddLine("    add r0, r0, r3");    // Add offset
                        AddLine("    str r2, [r0]");      // Store reference
                    }
                    break;
                case Code.Stfld:
                    if (instruction.Operand is IField storeField)
                    {
                        string fieldName = storeField.Name.Replace('.', '_');
                        int offset = 8; // This would be calculated based on field offset
                        if (_architecture == "x64")
                            AddLine($"    mov [rcx+{offset}], rdx");  // Assuming object in rcx, value in rdx
                        else if (_architecture == "x86")
                            AddLine($"    mov [eax+{offset}], ebx");  // Assuming object in eax, value in ebx
                        else if (_architecture == "arm64")
                            AddLine($"    str x1, [x0, #{offset}]");
                        else if (_architecture == "arm32")
                            AddLine($"    str r1, [r0, #{offset}]");
                    }
                    break;
                case Code.Stind_I:
                    if (_architecture == "x64")
                        AddLine("    mov qword ptr [rcx], rax");  // Store native int (64-bit) from RAX to address in RCX
                    else if (_architecture == "x86")
                        AddLine("    mov dword ptr [ecx], eax");  // Store native int (32-bit) from EAX to address in ECX
                    else if (_architecture == "arm64")
                        AddLine("    str x0, [x1]");  // Store native int (64-bit) from X0 to address in X1
                    else if (_architecture == "arm32")
                        AddLine("    str r0, [r1]");  // Store native int (32-bit) from R0 to address in R1
                    break;
                case Code.Stind_I1:
                    if (_architecture == "x64")
                        AddLine("    mov byte ptr [rcx], al");  // Store byte from AL to address in RCX
                    else if (_architecture == "x86")
                        AddLine("    mov byte ptr [ecx], al");  // Store byte from AL to address in ECX
                    else if (_architecture == "arm64")
                        AddLine("    strb w0, [x1]");  // Store byte from W0 to address in X1
                    else if (_architecture == "arm32")
                        AddLine("    strb r0, [r1]");  // Store byte from R0 to address in R1
                    break;
                case Code.Stind_I2:
                    if (_architecture == "x64")
                        AddLine("    mov word ptr [rcx], ax");  // Store word from AX to address in RCX
                    else if (_architecture == "x86")
                        AddLine("    mov word ptr [ecx], ax");  // Store word from AX to address in ECX
                    else if (_architecture == "arm64")
                        AddLine("    strh w0, [x1]");  // Store halfword from W0 to address in X1
                    else if (_architecture == "arm32")
                        AddLine("    strh r0, [r1]");  // Store halfword from R0 to address in R1
                    break;
                case Code.Stind_I4:
                    if (_architecture == "x64")
                        AddLine("    mov dword ptr [rcx], eax");  // Store dword from EAX to address in RCX
                    else if (_architecture == "x86")
                        AddLine("    mov dword ptr [ecx], eax");  // Store dword from EAX to address in ECX
                    else if (_architecture == "arm64")
                        AddLine("    str w0, [x1]");  // Store word from W0 to address in X1
                    else if (_architecture == "arm32")
                        AddLine("    str r0, [r1]");  // Store word from R0 to address in R1
                    break;
                case Code.Stind_I8:
                    if (_architecture == "x64")
                        AddLine("    mov qword ptr [rcx], rax");  // Store qword from RAX to address in RCX
                    else if (_architecture == "x86")
                    {
                        // On 32-bit x86, storing a 64-bit value requires two operations
                        AddLine("    mov dword ptr [ecx], eax");    // Store low dword
                        AddLine("    mov dword ptr [ecx+4], edx");  // Store high dword (assuming in EDX)
                    }
                    else if (_architecture == "arm64")
                        AddLine("    str x0, [x1]");  // Store doubleword from X0 to address in X1
                    else if (_architecture == "arm32")
                    {
                        // On 32-bit ARM, storing a 64-bit value requires two operations
                        AddLine("    str r0, [r1]");     // Store low word
                        AddLine("    str r1, [r1, #4]"); // Store high word (assuming in R1)
                    }
                    break;
                case Code.Stind_R4:
                    if (_architecture == "x64")
                        AddLine("    movss dword ptr [rcx], xmm0");  // Store single-precision float from XMM0 to address in RCX
                    else if (_architecture == "x86")
                        AddLine("    movss dword ptr [ecx], xmm0");  // Store single-precision float from XMM0 to address in ECX
                    else if (_architecture == "arm64")
                        AddLine("    str s0, [x1]");  // Store single-precision float from S0 to address in X1
                    else if (_architecture == "arm32")
                        AddLine("    vstr s0, [r1]");  // Store single-precision float from S0 to address in R1
                    break;
                case Code.Stind_R8:
                    if (_architecture == "x64")
                        AddLine("    movsd qword ptr [rcx], xmm0");  // Store double-precision float from XMM0 to address in RCX
                    else if (_architecture == "x86")
                        AddLine("    movsd qword ptr [ecx], xmm0");  // Store double-precision float from XMM0 to address in ECX
                    else if (_architecture == "arm64")
                        AddLine("    str d0, [x1]");  // Store double-precision float from D0 to address in X1
                    else if (_architecture == "arm32")
                        AddLine("    vstr d0, [r1]");  // Store double-precision float from D0 to address in R1
                    break;
                case Code.Stind_Ref:
                    if (_architecture == "x64")
                        AddLine("    mov qword ptr [rcx], rax");  // Store reference from RAX to address in RCX
                    else if (_architecture == "x86")
                        AddLine("    mov dword ptr [ecx], eax");  // Store reference from EAX to address in ECX
                    else if (_architecture == "arm64")
                        AddLine("    str x0, [x1]");  // Store reference from X0 to address in X1
                    else if (_architecture == "arm32")
                        AddLine("    str r0, [r1]");  // Store reference from R0 to address in R1
                    break;
                case Code.Stloc:
                    if (instruction.Operand is Local var)
                    {
                        int offset = (var.Index + 1) * 8;  // Calculate stack offset for local variable
                        if (_architecture == "x64")
                            AddLine($"    mov [rbp-{offset}], rax");  // Store from RAX to local variable
                        else if (_architecture == "x86")
                            AddLine($"    mov [ebp-{offset / 2}], eax");  // Store from EAX to local variable
                        else if (_architecture == "arm64")
                            AddLine($"    str x0, [x29, #-{offset}]");  // Store from X0 to local variable
                        else if (_architecture == "arm32")
                            AddLine($"    str r0, [r7, #-{offset / 2}]");  // Store from R0 to local variable
                    }
                    break;
                case Code.Stloc_0:
                    if (_architecture == "x64")
                        AddLine("    mov [rbp-8], rax");
                    else if (_architecture == "x86")
                        AddLine("    mov [ebp-4], eax");
                    else if (_architecture == "arm64")
                        AddLine("    str x0, [x29, #-8]");
                    else if (_architecture == "arm32")
                        AddLine("    str r0, [r7, #-4]");
                    break;
                case Code.Stloc_1:
                    if (_architecture == "x64")
                        AddLine("    mov [rbp-16], rax");
                    else if (_architecture == "x86")
                        AddLine("    mov [ebp-8], eax");
                    else if (_architecture == "arm64")
                        AddLine("    str x0, [x29, #-16]");
                    else if (_architecture == "arm32")
                        AddLine("    str r0, [r7, #-8]");
                    break;
                case Code.Stloc_2:
                    if (_architecture == "x64")
                        AddLine("    mov [rbp-24], rax");
                    else if (_architecture == "x86")
                        AddLine("    mov [ebp-12], eax");
                    else if (_architecture == "arm64")
                        AddLine("    str x0, [x29, #-24]");
                    else if (_architecture == "arm32")
                        AddLine("    str r0, [r7, #-12]");
                    break;
                case Code.Stloc_3:
                    if (_architecture == "x64")
                        AddLine("    mov [rbp-32], rax");
                    else if (_architecture == "x86")
                        AddLine("    mov [ebp-16], eax");
                    else if (_architecture == "arm64")
                        AddLine("    str x0, [x29, #-32]");
                    else if (_architecture == "arm32")
                        AddLine("    str r0, [r7, #-16]");
                    break;
                case Code.Stloc_S:
                    if (instruction.Operand is Local localVar)
                    {
                        int offset = (localVar.Index + 1) * 8; // Simple offset calculation
                        if (_architecture == "x64")
                            AddLine($"    mov [rbp-{offset}], rax");
                        else if (_architecture == "x86")
                            AddLine($"    mov [ebp-{offset / 2}], eax");
                        else if (_architecture == "arm64")
                            AddLine($"    str x0, [x29, #-{offset}]");
                        else if (_architecture == "arm32")
                            AddLine($"    str r0, [r7, #-{offset / 2}]");
                    }
                    break;
                case Code.Stobj:
                    if (instruction.Operand is ITypeDefOrRef typeDef2)
                    {
                        // Store object based on the size of the type
                        if (_architecture == "x64")
                        {
                            AddLine("    mov [rcx], rax");  // Assuming address in rcx, value in rax
                        }
                        else if (_architecture == "x86")
                        {
                            AddLine("    mov ecx, [esp+4]"); // Get destination address
                            AddLine("    mov [ecx], eax");   // Store value at address
                        }
                        else if (_architecture == "arm64")
                        {
                            AddLine("    str x0, [x1]");  // Value in x0, address in x1
                        }
                        else if (_architecture == "arm32")
                        {
                            AddLine("    str r0, [r1]");  // Value in r0, address in r1
                        }
                    }
                    break;
                case Code.Stsfld:
                    if (instruction.Operand is IField staticField)
                    {
                        string fieldName = staticField.Name.Replace('.', '_');
                        string className = staticField.DeclaringType.Name.Replace('.', '_');
                        string fieldLabel = $"{className}_{fieldName}";

                        if (_architecture == "x64")
                            AddLine($"    mov [rip+{fieldLabel}], rax");  // Store to static field using RIP-relative addressing
                        else if (_architecture == "x86")
                            AddLine($"    mov [{fieldLabel}], eax");  // Direct absolute addressing
                        else if (_architecture == "arm64")
                        {
                            AddLine($"    adrp x1, {fieldLabel}");  // Address of page containing label
                            AddLine($"    str x0, [x1, :lo12:{fieldLabel}]");  // Store with low 12 bits of offset
                        }
                        else if (_architecture == "arm32")
                        {
                            AddLine($"    ldr r1, ={fieldLabel}");  // Load address of static field
                            AddLine($"    str r0, [r1]");  // Store value to field
                        }
                    }
                    break;
                case Code.Sub:
                    if (_architecture == "x64")
                        AddLine("    sub rax, rbx");
                    else if (_architecture == "x86")
                        AddLine("    sub eax, ebx");
                    else if (_architecture == "arm64")
                        AddLine("    sub x0, x0, x1");
                    else if (_architecture == "arm32")
                        AddLine("    sub r0, r0, r1");
                    break;
                case Code.Sub_Ovf:
                    if (_architecture == "x64") {
                        AddLine("    sub rax, rbx");
                        AddLine("    jo overflow_exception");  // Jump if overflow
                    } else if (_architecture == "x86") {
                        AddLine("    sub eax, ebx");
                        AddLine("    jo overflow_exception");  // Jump if overflow
                    } else if (_architecture == "arm64") {
                        AddLine("    subs x0, x0, x1");
                        AddLine("    b.vs overflow_exception");  // Branch if overflow (V set)
                    } else if (_architecture == "arm32") {
                        AddLine("    subs r0, r0, r1");
                        AddLine("    bvs overflow_exception");  // Branch if overflow (V set)
                    }
                    break;
                case Code.Sub_Ovf_Un:
                    if (_architecture == "x64") {
                        AddLine("    sub rax, rbx");
                        AddLine("    jc overflow_exception");  // Jump if carry (unsigned borrow occurred)
                    } else if (_architecture == "x86") {
                        AddLine("    sub eax, ebx");
                        AddLine("    jc overflow_exception");  // Jump if carry (unsigned borrow occurred)
                    } else if (_architecture == "arm64") {
                        AddLine("    subs x0, x0, x1");
                        AddLine("    b.cc overflow_exception");  // Branch if carry clear (unsigned borrow)
                    } else if (_architecture == "arm32") {
                        AddLine("    subs r0, r0, r1");
                        AddLine("    bcc overflow_exception");  // Branch if carry clear (unsigned borrow)
                    }
                    break;
                case Code.Switch:
                    if (instruction.Operand is IList<Instruction> targets) {
                        // Generate jump table for switch statement
                        if (_architecture == "x64") {
                            // Bounds check
                            AddLine($"    cmp rax, {targets.Count - 1}");
                            AddLine("    ja switch_default");  // Jump if above the max index
                            
                            // Use indexed jump
                            AddLine("    lea rbx, [switch_table]");
                            AddLine("    mov rbx, [rbx + rax*8]");  // Get jump target address
                            AddLine("    jmp rbx");
                            
                            // Define the jump table (would need to be in the data section)
                            AddLine("switch_table:");
                            for (int i = 0; i < targets.Count; i++) {
                                AddLine($"    dq IL_{targets[i].Offset:X4}");
                            }
                            AddLine("switch_default:");
                        } else if (_architecture == "x86") {
                            // Bounds check
                            AddLine($"    cmp eax, {targets.Count - 1}");
                            AddLine("    ja switch_default");
                            
                            // Use indexed jump
                            AddLine("    mov ebx, eax");
                            AddLine("    mov ebx, [switch_table + ebx*4]");
                            AddLine("    jmp ebx");
                            
                            // Define the jump table
                            AddLine("switch_table:");
                            for (int i = 0; i < targets.Count; i++) {
                                AddLine($"    dd IL_{targets[i].Offset:X4}");
                            }
                            AddLine("switch_default:");
                        } else if (_architecture == "arm64") {
                            // Bounds check
                            AddLine($"    cmp x0, #{targets.Count - 1}");
                            AddLine("    b.hi switch_default");
                            
                            // Use indexed jump
                            AddLine("    adr x1, switch_table");
                            AddLine("    add x1, x1, x0, lsl #3");  // Each entry is 8 bytes
                            AddLine("    ldr x1, [x1]");
                            AddLine("    br x1");
                            
                            // Define the jump table
                            AddLine("switch_table:");
                            for (int i = 0; i < targets.Count; i++) {
                                AddLine($"    .quad IL_{targets[i].Offset:X4}");
                            }
                            AddLine("switch_default:");
                        } else if (_architecture == "arm32") {
                            // Bounds check
                            AddLine($"    cmp r0, #{targets.Count - 1}");
                            AddLine("    bhi switch_default");
                            
                            // Use indexed jump
                            AddLine("    adr r1, switch_table");
                            AddLine("    ldr r1, [r1, r0, lsl #2]");  // Each entry is 4 bytes
                            AddLine("    bx r1");
                            
                            // Define the jump table
                            AddLine("switch_table:");
                            for (int i = 0; i < targets.Count; i++) {
                                AddLine($"    .word IL_{targets[i].Offset:X4}");
                            }
                            AddLine("switch_default:");
                        }
                    }
                    break;
            }
        }

        public string GenerateAssembly()
        {
            // Join all accumulated assembly lines with newlines
            return string.Join(Environment.NewLine, _lines);
        }
    }
}

