using System;
using System.Diagnostics;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Threading;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.Collections.Blobs
{
    [StructLayout(LayoutKind.Explicit, Size = 16)]
    internal unsafe struct BlobAssetBatch
    {
        [FieldOffset(0)]
        private int TotalDataSize;
        [FieldOffset(4)]
        private int BlobAssetHeaderCount;
        [FieldOffset(8)]
        private int RefCount;
        [FieldOffset(12)]
        private int Padding;

        internal static BlobAssetBatch CreateForSerialize(int blobAssetCount, int totalDataSize)
        {
            return new BlobAssetBatch
            {
                BlobAssetHeaderCount = blobAssetCount,
                TotalDataSize = totalDataSize,
                RefCount = 1,
                Padding = 0
            };
        }

        internal static BlobAssetBatch* CreateFromMemory(void* buffer, int expectedTotalDataSize)
        {
            var batch = (BlobAssetBatch*)buffer;

#if ENABLE_UNITY_COLLECTIONS_CHECKS || UNITY_DOTS_DEBUG
            if (batch->TotalDataSize != expectedTotalDataSize)
                throw new System.ArgumentException($"TotalSize '{batch->TotalDataSize}' and expected Total size '{expectedTotalDataSize}' are out of sync");
            if (batch->RefCount != 1)
                throw new System.ArgumentException("BlobAssetBatch.Refcount must be 1 on deserialized");

            var header = (BlobAssetHeader*)(batch + 1);
            for (int i = 0; i != batch->BlobAssetHeaderCount; i++)
            {
                header->ValidationPtr = header + 1;
                if (header->Allocator.ToAllocator != Allocator.None)
                    throw new System.ArgumentException("Blob Allocator should be Allocator.None");
                header = (BlobAssetHeader*)(((byte*)(header + 1)) + header->Length);
            }
            header--;

            if (header == (byte*)batch + batch->TotalDataSize)
                throw new System.ArgumentException("");
#endif

            return batch;
        }

        public static void Retain(BlobAssetBatch* batch)
        {
            Interlocked.Increment(ref batch->RefCount);
        }

        public static void Release(BlobAssetBatch* batch)
        {
            int newRefCount = Interlocked.Decrement(ref batch->RefCount);
            if (newRefCount <= 0)
            {
                // Debug.Log("Freeing blob");

#if ENABLE_UNITY_COLLECTIONS_CHECKS || UNITY_DOTS_DEBUG
                if (newRefCount < 0)
                    throw new InvalidOperationException("BlobAssetBatch refcount is less than zero. It has been corrupted.");

                if (batch->TotalDataSize == 0)
                    throw new InvalidOperationException("BlobAssetBatch has been corrupted. Likely it has already been unloaded or released.");

                var header = (BlobAssetHeader*)(batch + 1);
                for (int i = 0; i != batch->BlobAssetHeaderCount; i++)
                {
                    if (header->ValidationPtr != (header + 1))
                        throw new InvalidOperationException("The BlobAssetReference has been corrupted. Likely it has already been unloaded or released.");

                    header->Invalidate();
                    header = (BlobAssetHeader*)(((byte*)(header + 1)) + header->Length);
                }
                header--;

                if (header == (byte*)batch + batch->TotalDataSize)
                    throw new InvalidOperationException("BlobAssetBatch has been corrupted. Likely it has already been unloaded or released.");

                batch->TotalDataSize = 0;
                batch->BlobAssetHeaderCount = 0;
#endif
                Memory.Unmanaged.Free(batch, Allocator.Persistent);
            }
        }
    }

    readonly unsafe struct BlobAssetPtr : IEquatable<BlobAssetPtr>
    {
        public readonly BlobAssetHeader* Header;
        public void* Data => Header + 1;
        public int Length => Header->Length;
        public ulong Hash => Header->Hash;

        public BlobAssetPtr(BlobAssetHeader* header)
            => Header = header;

        public bool Equals(BlobAssetPtr other)
            => Header == other.Header;

        public override int GetHashCode()
        {
            BlobAssetHeader* onStack = Header;
            return (int)math.hash(&onStack, sizeof(BlobAssetHeader*));
        }
    }

    // TODO: For now the size of BlobAssetHeader needs to be multiple of 16 to ensure alignment of blob assets
    // TODO: Add proper alignment support to blob assets
    // TODO: Reduce the size of the header at runtime or remove it completely
    [StructLayout(LayoutKind.Explicit, Size = 32)]
    unsafe struct BlobAssetHeader
    {
        [FieldOffset(0)] public void* ValidationPtr;
        [FieldOffset(8)] public int Length;
        [FieldOffset(12)] public AllocatorManager.AllocatorHandle Allocator;
        [FieldOffset(16)] public ulong Hash;
        [FieldOffset(24)] private ulong Padding;

        internal static BlobAssetHeader CreateForSerialize(int length, ulong hash)
        {
            return new BlobAssetHeader
            {
                ValidationPtr = null,
                Length = length,
                Allocator = Unity.Collections.Allocator.None,
                Hash = hash,
                Padding = 0
            };
        }

        public void Invalidate()
        {
            ValidationPtr = (void*)0xdddddddddddddddd;
        }
    }

    [StructLayout(LayoutKind.Explicit, Size = 8)]
    internal unsafe struct BlobAssetReferenceData : IEquatable<BlobAssetReferenceData>
    {
        [NativeDisableUnsafePtrRestriction]
        [FieldOffset(0)]
        public byte* m_Ptr;


        /// <summary>
        /// This field overlaps m_Ptr similar to a C union.
        /// It is an internal (so we can initialize the struct) field which
        /// is here to force the alignment of BlobAssetReferenceData to be 8-bytes.
        /// </summary>
        [FieldOffset(0)]
        internal long m_Align8Union;

        internal BlobAssetHeader* Header
        {
            get
            {
                ThrowIfNull();
                return (BlobAssetHeader*)m_Ptr - 1;
            }
        }

        /// <summary>
        /// This member is exposed to Unity.Properties to support EqualityComparison and Serialization within managed objects.
        /// </summary>
        /// <remarks>
        /// This member is used to expose the value of the <see cref="m_Ptr"/> to properties (which does not handle pointers by default).
        ///
        /// It's used for two managed object cases.
        ///
        /// 1) EqualityComparison - The equality comparison visitor will encounter this member and compare the value (i.e. blob address).
        ///
        ///
        /// 2) Serialization - Before serialization, the <see cref="m_Ptr"/> field is patched with a serialized hash. The visitor encounters this member
        ///                    and writes/reads back the value. The value is then patched back to the new ptr.
        ///
        /// 3) ManagedObjectClone - When cloning managed objects Unity.Properties does not have access to the internal pointer field. This property is used to copy the bits for this struct.
        /// </remarks>
        // ReSharper disable once UnusedMember.Local
        [Properties.CreateProperty]
        long SerializedHash
        {
            get => m_Align8Union;
            set => m_Align8Union = value;
        }

        [BurstDiscard]
        void ValidateNonBurst()
        {
            void* validationPtr = null;
            try
            {
                // Try to read ValidationPtr, this might throw if the memory has been unmapped
                validationPtr = Header->ValidationPtr;
            }
            catch (Exception)
            {
            }

            if (!(validationPtr == m_Ptr || IsProtected))
            {
                throw new InvalidOperationException("The BlobAssetReference is not valid. Likely it has already been unloaded or released.");
            }
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        void ValidateBurst()
        {
            void* validationPtr = Header->ValidationPtr;
            if (!(validationPtr == m_Ptr || IsProtected))
            {
                throw new InvalidOperationException("The BlobAssetReference is not valid. Likely it has already been unloaded or released.");
            }
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        public void ValidateNotNull()
        {
            ThrowIfNull();

            ValidateNonBurst();
            ValidateBurst();
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        void ThrowIfNull()
        {
            if (m_Ptr == null)
                throw new NullReferenceException("The BlobAssetReference is null.");
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        public void ValidateAllowNull()
        {
            if (m_Ptr == null)
                return;

            ValidateNonBurst();
            ValidateBurst();
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        private void ValidateNotDeserialized()
        {
            if (Header->Allocator.ToAllocator == Allocator.None)
                throw new InvalidOperationException("It's not possible to release a blob asset reference that was deserialized. It will be automatically released when the scene is unloaded ");
            Header->Invalidate();
        }

        public void Dispose()
        {
            ValidateNotNull();
            ValidateNotProtected();
            ValidateNotDeserialized();
            Memory.Unmanaged.Free(Header, Header->Allocator);
            m_Ptr = null;
        }

        public bool Equals(BlobAssetReferenceData other)
        {
            return m_Ptr == other.m_Ptr;
        }

        public override int GetHashCode()
        {
#if UNITY_64
            int low = (int)m_Ptr;
            int hi = (int)m_Ptr >> 32;
            return (low * 397) ^ hi;
#else
            return (int) m_Ptr;
#endif
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        internal void ProtectAgainstDisposal()
        {
            ValidateNotNull();

            if (IsProtected)
            {
                throw new InvalidOperationException("Cannot protect this BlobAssetReference, it is already protected.");
            }
#if UNITY_64
            Header->ValidationPtr = (void*)~(long)m_Ptr;
#else
            Header->ValidationPtr = (void*)~(int)m_Ptr;
#endif
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        internal void UnprotectAgainstDisposal()
        {
            ValidateNotNull();

            if (!IsProtected)
            {
                throw new InvalidOperationException("Cannot unprotect this BlobAssetReference, it is not protected.");
            }

            Header->ValidationPtr = m_Ptr;
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        void ValidateNotProtected()
        {
            if (IsProtected)
            {
                throw new InvalidOperationException("This BlobAssetReference is owned by a BlobAssetStore and cannot be disposed directly.");
            }
        }

        bool IsProtected
        {
            get
            {
                ThrowIfNull();

#if UNITY_64
                return Header->ValidationPtr == (void*)~(long)m_Ptr;
#else
                return Header->ValidationPtr == (void*)~(int)m_Ptr;
#endif
            }
        }
    }

    /// <summary>
    /// A reference to a blob asset stored in unmanaged memory.
    /// </summary>
    /// <remarks>Create a blob asset using a <see cref="BlobBuilder"/> or by deserializing a serialized blob asset.</remarks>
    /// <typeparam name="T">The struct data type defining the data structure of the blob asset.</typeparam>
    [DebuggerDisplay(nameof(BlobAssetReference<T>))]
    public unsafe struct BlobAssetReference<T> : IDisposable, IEquatable<BlobAssetReference<T>>
        where T : unmanaged
    {
        [Properties.CreateProperty]
        internal BlobAssetReferenceData m_data;
        /// <summary>
        /// Reports whether this instance references a valid blob asset.
        /// </summary>
        /// <value>True, if this instance references a valid blob instance.</value>
        public bool IsCreated
        {
            get { return m_data.m_Ptr != null; }
        }

        /// <summary>
        /// Provides an unsafe pointer to the blob asset data.
        /// </summary>
        /// <remarks>You can only use unsafe pointers in [unsafe contexts].
        /// [unsafe contexts]: https://docs.microsoft.com/en-us/dotnet/csharp/language-reference/language-specification/unsafe-code
        /// </remarks>
        /// <returns>An unsafe pointer. The pointer is null for invalid BlobAssetReference instances.</returns>
        public void* GetUnsafePtr()
        {
            m_data.ValidateAllowNull();
            return m_data.m_Ptr;
        }

        /// <summary>
        /// Destroys the referenced blob asset and frees its memory.
        /// </summary>
        /// <exception cref="InvalidOperationException">Thrown if you attempt to dispose a blob asset that loaded as
        /// part of a scene or subscene.</exception>
        public void Dispose()
        {
            m_data.Dispose();
        }


        /// <summary>
        /// A reference to the blob asset data, a struct of type T that is stored in the blob asset.
        /// </summary>
        /// <remarks>The property is a
        /// <see href="https://docs.microsoft.com/en-us/dotnet/csharp/programming-guide/classes-and-structs/ref-returns">
        /// reference return</see>.</remarks>
        /// <value>The root data structure of the blob asset data.</value>
        public ref T Value
        {
            get
            {
                m_data.ValidateNotNull();
                return ref UnsafeUtility.AsRef<T>(m_data.m_Ptr);
            }
        }


        /// <summary>
        /// Creates a blob asset from a pointer to data and a specified size.
        /// </summary>
        /// <remarks>The blob asset is created in unmanaged memory. Call <see cref="Dispose"/> to free the asset memory
        /// when it is no longer needed. This function can only be used in an [unsafe context].
        /// [unsafe context]: https://docs.microsoft.com/en-us/dotnet/csharp/language-reference/language-specification/unsafe-code
        /// </remarks>
        /// <param name="ptr">A pointer to the buffer containing the data to store in the blob asset.</param>
        /// <param name="length">The length of the buffer in bytes.</param>
        /// <returns>A reference to newly created blob asset.</returns>
        /// <seealso cref="BlobBuilder"/>
        public static BlobAssetReference<T> Create(void* ptr, int length)
        {
            byte* buffer =
                (byte*)Memory.Unmanaged.Allocate(sizeof(BlobAssetHeader) + length, 16, Allocator.Persistent);
            UnsafeUtility.MemCpy(buffer + sizeof(BlobAssetHeader), ptr, length);

            BlobAssetHeader* header = (BlobAssetHeader*)buffer;
            *header = new BlobAssetHeader();

            header->Length = length;
            header->Allocator = Allocator.Persistent;

            // @TODO use 64bit hash
            header->Hash = math.hash(ptr, length);

            BlobAssetReference<T> blobAssetReference;
            blobAssetReference.m_data.m_Align8Union = 0;
            header->ValidationPtr = blobAssetReference.m_data.m_Ptr = buffer + sizeof(BlobAssetHeader);
            return blobAssetReference;
        }

        /// <summary>
        /// Creates a blob asset from a byte array.
        /// </summary>
        /// <remarks>The blob asset is created in unmanaged memory. Call <see cref="Dispose"/> to free the asset memory
        /// when it is no longer needed. This function can only be used in an [unsafe context].
        /// [unsafe context]: https://docs.microsoft.com/en-us/dotnet/csharp/language-reference/language-specification/unsafe-code
        /// </remarks>
        /// <param name="data">The byte array containing the data to store in the blob asset.</param>
        /// <returns>A reference to newly created blob asset.</returns>
        /// <seealso cref="BlobBuilder"/>
        public static BlobAssetReference<T> Create(byte[] data)
        {
            fixed (byte* ptr = &data[0])
            {
                return Create(ptr, data.Length);
            }
        }

        /// <summary>
        /// Creates a blob asset from an instance of a struct.
        /// </summary>
        /// <remarks>The struct must only contain blittable fields (primitive types, fixed-length arrays, or other structs
        /// meeting these same criteria). The blob asset is created in unmanaged memory. Call <see cref="Dispose"/> to
        /// free the asset memory when it is no longer needed. This function can only be used in an [unsafe context].
        /// [unsafe context]: https://docs.microsoft.com/en-us/dotnet/csharp/language-reference/language-specification/unsafe-code</remarks>
        /// <param name="value">An instance of <typeparamref name="T"/>.</param>
        /// <returns>A reference to newly created blob asset.</returns>
        /// <seealso cref="BlobBuilder"/>
        public static BlobAssetReference<T> Create(T value)
        {
            return Create(UnsafeUtility.AddressOf(ref value), UnsafeUtility.SizeOf<T>());
        }

        /// <summary>
        /// Construct a BlobAssetReference from the blob data
        /// </summary>
        /// <param name="blobData">The blob data to attach to the returned object</param>
        /// <returns>The created BlobAssetReference</returns>
        internal static BlobAssetReference<T> Create(BlobAssetReferenceData blobData)
        {
            return new BlobAssetReference<T> { m_data = blobData };
        }

        /// <summary>
        /// Reads bytes from a binary reader, validates the expected serialized version, and deserializes them into a new blob asset.
        /// </summary>
        /// <param name="binaryReader">The reader for the blob data path</param>
        /// <param name="version">Expected version number of the blob data.</param>
        /// <param name="result">The resulting BlobAssetReference if the data was read successful.</param>
        /// <typeparam name="U">The type of binary reader</typeparam>
        /// <returns>True if the data was successfully read, false otherwise</returns>
        public static bool TryRead<U>(U binaryReader, int version, out BlobAssetReference<T> result)
        where U : BinaryReader
        {
            var storedVersion = binaryReader.ReadInt();
            if (storedVersion != version)
            {
                result = default;
                return false;
            }
            result = binaryReader.Read<T>();
            return true;
        }

        /// <summary>
        /// Reads bytes from a fileName, validates the expected serialized version, and deserializes them into a new blob asset.
        /// </summary>
        /// <param name="path">The path of the blob data to read.</param>
        /// <param name="version">Expected version number of the blob data.</param>
        /// <param name="result">The resulting BlobAssetReference if the data was read successful.</param>
        /// <returns>A bool if the read was successful or not.</returns>
        public static bool TryRead(string path, int version, out BlobAssetReference<T> result)
        {
            if (string.IsNullOrEmpty(path))
            {
                result = default;
                return false;
            }
            using (var binaryReader = new StreamBinaryReader(path, UnsafeUtility.SizeOf<T>() + sizeof(int)))
            {
                return TryRead(binaryReader, version, out result);
            }
        }

        /// <summary>
        /// Reads bytes from a buffer, validates the expected serialized version, and deserializes them into a new blob asset.
        /// The returned blob reuses the data from the passed in pointer and is only valid as long as the buffer stays allocated.
        /// Also the returned blob asset reference can not be disposed.
        /// </summary>
        /// <param name="data">A pointer to the buffer containing the serialized blob data.</param>
        /// <param name="size">Size in bytes of the buffer containing the serialized blob data.</param>
        /// <param name="version">Expected version number of the blob data.</param>
        /// <param name="result">The resulting BlobAssetReference if the data was read successful.</param>
        /// <param name="numBytesRead">Number of bytes of the data buffer that are read.</param>
        /// <returns>A bool if the read was successful or not.</returns>
        internal static unsafe bool TryReadInplace(byte* data, long size, int version, out BlobAssetReference<T> result, out int numBytesRead)
        {
            result = default;
            numBytesRead = 0;

            if (size < sizeof(int) + sizeof(BlobAssetHeader))
                return false;

            var storedVersion = *(int*)data;
            if (storedVersion != version)
                return false;

            ref var header = ref *((BlobAssetHeader*)(data + sizeof(int)));

            if (size < sizeof(int) + sizeof(BlobAssetHeader) + header.Length)
                return false;

            numBytesRead = sizeof(int) + sizeof(BlobAssetHeader) + header.Length;

            var buffer = data + sizeof(int);
            header.ValidationPtr = buffer + sizeof(BlobAssetHeader);

            BlobAssetReference<T> blobAssetReference;
            blobAssetReference.m_data.m_Align8Union = 0;
            blobAssetReference.m_data.m_Ptr = buffer + sizeof(BlobAssetHeader);
            result = blobAssetReference;
            return true;
        }

        /// <summary>
        /// Writes the blob data to a path with serialized version.
        /// </summary>
        /// <param name="writer">The binary writer to write the blob with.</param>
        /// <param name="builder">The BlobBuilder containing the blob to write.</param>
        /// <param name="version">Serialized version number of the blob data.</param>
        /// <typeparam name="U">The type of binary writer</typeparam>
        public static void Write<U>(U writer, BlobBuilder builder, int version)
        where U : BinaryWriter
        {
            using (var asset = builder.CreateBlobAssetReference<T>(Allocator.TempJob))
            {
                writer.Write(version);
                writer.Write(asset);
            }
        }

        /// <summary>
        /// Writes the blob data to a path with serialized version.
        /// </summary>
        /// <param name="builder">The BlobBuilder containing the blob to write.</param>
        /// <param name="path">The path to write the blob data.</param>
        /// <param name="version">Serialized version number of the blob data.</param>
        public static void Write(BlobBuilder builder, string path, int version)
        {
            using (var writer = new StreamBinaryWriter(path))
            {
                Write(writer, builder, version);
            }
        }

        /// <summary>
        /// A "null" blob asset reference that can be used to test if a BlobAssetReference instance
        /// </summary>
        public static BlobAssetReference<T> Null => new BlobAssetReference<T>();

        /// <summary>
        /// Two BlobAssetReferences are equal when they reference the same data.
        /// </summary>
        /// <param name="lhs">The BlobAssetReference on the left side of the operator.</param>
        /// <param name="rhs">The BlobAssetReference on the right side of the operator.</param>
        /// <returns>True, if both references point to the same data or if both are <see cref="Null"/>.</returns>
        public static bool operator ==(BlobAssetReference<T> lhs, BlobAssetReference<T> rhs)
        {
            return lhs.m_data.m_Ptr == rhs.m_data.m_Ptr;
        }

        /// <summary>
        /// Two BlobAssetReferences are not equal unless they reference the same data.
        /// </summary>
        /// <param name="lhs">The BlobAssetReference on the left side of the operator.</param>
        /// <param name="rhs">The BlobAssetReference on the right side of the operator.</param>
        /// <returns>True, if the references point to different data in memory or if one is <see cref="Null"/>.</returns>
        public static bool operator !=(BlobAssetReference<T> lhs, BlobAssetReference<T> rhs)
        {
            return lhs.m_data.m_Ptr != rhs.m_data.m_Ptr;
        }

        /// <summary>
        /// Two BlobAssetReferences are equal when they reference the same data.
        /// </summary>
        /// <param name="other">The reference to compare to this one.</param>
        /// <returns>True, if both references point to the same data or if both are <see cref="Null"/>.</returns>
        public bool Equals(BlobAssetReference<T> other)
        {
            return m_data.Equals(other.m_data);
        }

        /// <summary>
        /// Two BlobAssetReferences are equal when they reference the same data.
        /// </summary>
        /// <param name="obj">The object to compare to this reference</param>
        /// <returns>True, if the object is a BlobAssetReference instance that references to the same data as this one,
        /// or if both objects are <see cref="Null"/> BlobAssetReference instances.</returns>
        public override bool Equals(object obj)
        {
            return this == (BlobAssetReference<T>)obj;
        }

        /// <summary>
        /// Generates the hash code for this object.
        /// </summary>
        /// <returns>A standard C# value-type hash code.</returns>
        public override int GetHashCode()
        {
            return m_data.GetHashCode();
        }

        internal BlobAssetPtr ToBlobAssetPtr() => new BlobAssetPtr(m_data.Header);
    }

    /// <summary>
    /// A pointer referencing a struct, array, or field inside a blob asset.
    /// </summary>
    /// <typeparam name="T">The data type of the referenced object.</typeparam>
    /// <seealso cref="BlobBuilder"/>
    [MayOnlyLiveInBlobStorage]
    [DebuggerDisplay("Cannot display the value of a " + nameof(BlobPtr<T>) + " by itself. Please inspect the root BlobAssetReference instead.")]
    public unsafe struct BlobPtr<T> where T : struct
    {
        internal int m_OffsetPtr;

        /// <summary>
        /// Returns 'true' if this is a valid pointer (not null)
        /// </summary>
        public bool IsValid => m_OffsetPtr != 0;

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        private void AssertIsValid()
        {
            if (!IsValid)
                throw new System.InvalidOperationException("The accessed BlobPtr hasn't been allocated.");
        }

        /// <summary>
        /// The value, of type <typeparamref name="T"/> to which the pointer refers.
        /// </summary>
        /// <remarks>The property is a
        /// <see href="https://docs.microsoft.com/en-us/dotnet/csharp/programming-guide/classes-and-structs/ref-returns">
        /// reference return</see>.</remarks>
        /// <exception cref="InvalidOperationException">Thrown if the pointer does not reference a valid instance of
        /// a data type.</exception>
        public ref T Value
        {
            get
            {
                AssertIsValid();
                fixed (int* thisPtr = &m_OffsetPtr)
                {
                    return ref UnsafeUtility.AsRef<T>((byte*)thisPtr + m_OffsetPtr);
                }
            }
        }

        /// <summary>
        /// Provides an unsafe pointer to the referenced data.
        /// </summary>
        /// <remarks>You can only use unsafe pointers in [unsafe contexts].
        /// [unsafe contexts]: https://docs.microsoft.com/en-us/dotnet/csharp/language-reference/language-specification/unsafe-code
        /// </remarks>
        /// <returns>An unsafe pointer.</returns>
        public void* GetUnsafePtr()
        {
            if (m_OffsetPtr == 0)
                return null;

            fixed (int* thisPtr = &m_OffsetPtr)
            {
                return (byte*)thisPtr + m_OffsetPtr;
            }
        }
    }

    /// <summary>
    ///  An immutable array of value types stored in a blob asset.
    /// </summary>
    /// <remarks>When creating a blob asset, use the <see cref="BlobBuilderArray{T}"/> provided by a
    /// <see cref="BlobBuilder"/> instance to set the array elements.</remarks>
    /// <typeparam name="T">The data type of the elements in the array. Must be a struct or other value type.</typeparam>
    /// <seealso cref="BlobBuilder"/>
    [MayOnlyLiveInBlobStorage]
    [DebuggerDisplay("Cannot display the value of a " + nameof(BlobArray<T>) + " by itself. Please inspect the root BlobAssetReference instead.")]
    public unsafe struct BlobArray<T> where T : struct
    {
        internal int m_OffsetPtr;
        internal int m_Length;

        /// <summary>
        /// The number of elements in the array.
        /// </summary>
        public int Length
        {
            get { return m_Length; }
        }

        /// <summary>
        /// Provides an unsafe pointer to the array data.
        /// </summary>
        /// <remarks>You can only use unsafe pointers in [unsafe contexts].
        /// [unsafe contexts]: https://docs.microsoft.com/en-us/dotnet/csharp/language-reference/language-specification/unsafe-code
        /// </remarks>
        /// <returns>An unsafe pointer.</returns>
        public void* GetUnsafePtr()
        {
            // for an unallocated array this will return an invalid pointer which is ok since it
            // should never be accessed as Length will be 0
            fixed (int* thisPtr = &m_OffsetPtr)
            {
                return (byte*)thisPtr + m_OffsetPtr;
            }
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        private void AssertIndexInRange(int index)
        {
            if ((uint)index >= (uint)m_Length)
                throw new System.IndexOutOfRangeException(string.Format("Index {0} is out of range Length {1}",
                    index, m_Length));
        }

        /// <summary>
        /// The element of the array at the <paramref name="index"/> position.
        /// </summary>
        /// <param name="index">The array index.</param>
        /// <remarks>The array element is a
        /// <see href="https://docs.microsoft.com/en-us/dotnet/csharp/programming-guide/classes-and-structs/ref-returns">
        /// reference return</see>.</remarks>
        /// <exception cref="IndexOutOfRangeException">Thrown if <paramref name="index"/> is out of bounds.</exception>
        public ref T this[int index]
        {
            get
            {
                AssertIndexInRange(index);

                fixed (int* thisPtr = &m_OffsetPtr)
                {
                    return ref UnsafeUtility.ArrayElementAsRef<T>((byte*)thisPtr + m_OffsetPtr, index);
                }
            }
        }


        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        private static void ThrowOnCopyOfBlobArrayFields(Type t)
        {
            ThrowOnCopyOfBlobArrayFieldsRecursive(t, t);
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        private static void ThrowOnCopyOfBlobArrayFieldsRecursive(Type t, Type rootType)
        {
            if (t.IsPointer || t.IsPrimitive || t.IsEnum || t == typeof(string))
                return;

            if (t.IsGenericType)
            {
                if (t.GetGenericTypeDefinition() == typeof(BlobArray<>))
                    throw new InvalidOperationException($"Type {rootType.Name} cannot be copied because it contains nested fields of type BlobArray<>.");
                if (t.GetGenericTypeDefinition() == typeof(BlobPtr<>))
                    throw new InvalidOperationException($"Type {rootType.Name} cannot be copied because it contains nested fields of type BlobPtr<>.");
            }

            var fields = t.GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
            for (int i = 0, count = fields?.Length ?? 0; i < count; ++i)
            {
                var field = fields[i];
                ThrowOnCopyOfBlobArrayFieldsRecursive(field.FieldType, rootType);
            }
        }

        /// <summary>
        /// Copies the elements of this BlobArray to a new managed array.
        /// </summary>
        /// <returns>An array containing copies of the elements of the BlobArray.</returns>
        /// <exception cref="InvalidOperationException">Throws InvalidOperationException if the array type contains
        /// nested <see cref="BlobArray{T}"/>, <see cref="BlobString"/> or <see cref="BlobPtr{T}"/> fields.</exception>
        public T[] ToArray()
        {
            ThrowOnCopyOfBlobArrayFields(typeof(T));

            var result = new T[m_Length];
            if (m_Length > 0)
            {
                var src = GetUnsafePtr();

                var handle = GCHandle.Alloc(result, GCHandleType.Pinned);
                var addr = handle.AddrOfPinnedObject();

                UnsafeUtility.MemCpy((void*)addr, src, m_Length * UnsafeUtility.SizeOf<T>());

                handle.Free();
            }
            return result;
        }
    }

    /// <summary>
    /// Use this attribute if you have structs that use offset pointers that are only valid when they live inside the blob storage.
    /// It will turn ensure a compiler error is generated for every time a reference to the struct is copied, or a field is read
    /// from a reference to the struct this attribute is applied on.
    /// </summary>
    public class MayOnlyLiveInBlobStorageAttribute : Attribute
    {
    }

    /// <summary>
    /// An immutable, variable-length string stored in a blob asset.
    /// </summary>
    /// <seealso cref="BlobBuilder"/>
    [MayOnlyLiveInBlobStorage]
    [DebuggerDisplay("Cannot display the value of a " + nameof(BlobString) + " by itself. Please inspect the root BlobAssetReference instead.")]
    public unsafe struct BlobString
    {
        internal BlobArray<byte> Data;
        /// <summary>
        /// The length of the string in UTF-8 bytes.
        /// </summary>
        public int Length
        {
            get { return math.max(0, Data.Length - 1); } // it's null-terminated, but we don't count the trailing null.
        }

        /// <summary>
        /// Converts this BlobString to a standard C# <see cref="string"/>.
        /// </summary>
        /// <returns>The C# string.</returns>
        public new string ToString() => ToString((byte*)Data.GetUnsafePtr(), Length);

        internal static string ToString(byte* data, int lengthInBytes)
        {
            var utf16Capacity = math.max(1, lengthInBytes * 2);
            var c = stackalloc char[utf16Capacity];
            Unicode.Utf8ToUtf16(data, lengthInBytes, c, out var utf16Length, utf16Capacity);
            return new String(c, 0, utf16Length);
        }

        /// <summary>
        /// Copies the characters from a BlobString to a native container
        /// </summary>
        /// <param name="dest">The destination BlobString.</param>
        /// <typeparam name="T">The type of native container to copy to</typeparam>
        /// <returns>None if the copy fully completes. Otherwise, returns Overflow.</returns>
        public ConversionError CopyTo<T>(ref T dest) where T : INativeList<byte>
        {
            byte* srcBuffer = (byte*)Data.GetUnsafePtr();
            int srcLength = Length;
            dest.Length = srcLength;
            byte* destBuffer = (byte*)UnsafeUtility.AddressOf(ref dest.ElementAt(0));
            int destCapacity = dest.Capacity;
            var err = Unicode.Utf8ToUtf8(srcBuffer, srcLength, destBuffer, out var destLength, destCapacity);
            dest.Length = destLength;
            return err;
        }
    }

    /// <summary>
    /// Extensions that allow the creation of <see cref="BlobString"/> instances by a <see cref="BlobBuilder"/>.
    /// </summary>
    public static class BlobStringExtensions
    {
        /// <summary>
        /// Allocates memory to store the string in a blob asset and copies the string data into it.
        /// </summary>
        /// <param name="builder">The BlobBuilder instance building the blob asset.</param>
        /// <param name="blobStr">A reference to the field in the blob asset that will store the string. This
        /// function allocates memory for that field and sets the string value.</param>
        /// <param name="value">The string to copy into the blob asset.</param>
        public static unsafe void AllocateString(ref this BlobBuilder builder, ref BlobString blobStr, string value)
        {
            fixed (char* c = value)
            {
                var utf8Capacity = value.Length * 2 + 1;
                byte* b = (byte*)UnsafeUtility.Malloc(utf8Capacity, 1, Allocator.Temp);
                Unicode.Utf16ToUtf8(c, value.Length, b, out int utf8Length, utf8Capacity);
                b[utf8Length] = 0;
                var res = builder.Allocate(ref blobStr.Data, utf8Length + 1);
                UnsafeUtility.MemCpy(res.GetUnsafePtr(), b, utf8Length + 1);
            }
        }

        /// <summary>
        /// Allocates memory to store the string in a blob asset and copies the string data into it.
        /// </summary>
        /// <param name="builder">The BlobBuilder instance building the blob asset.</param>
        /// <param name="blobStr">A reference to the field in the blob asset that will store the string. This
        /// function allocates memory for that field and sets the string value.</param>
        /// <param name="value">The string to copy into the blob asset.</param>
        /// <typeparam name="T">The type of native container that contains the source string</typeparam>
        public static unsafe void AllocateString<T>(ref this BlobBuilder builder, ref BlobString blobStr, ref T value) where T : INativeList<byte>
        {
            var utf8Length = value.Length;
            byte* utf8Bytes = (byte*)UnsafeUtility.AddressOf(ref value.ElementAt(0));
            var res = builder.Allocate(ref blobStr.Data, utf8Length + 1);
            UnsafeUtility.MemCpy(res.GetUnsafePtr(), utf8Bytes, utf8Length + 1);
        }
    }

    /// <summary>
    /// Extensions for supporting serialization and deserialization of blob assets.
    /// </summary>
    public static class BlobAssetSerializeExtensions
    {
        /// <summary>
        /// Serializes the blob asset data and writes the bytes to a <see cref="BinaryWriter"/> instance.
        /// </summary>
        /// <param name="binaryWriter">An implementation of the BinaryWriter interface.</param>
        /// <param name="blob">A reference to the blob asset to serialize.</param>
        /// <typeparam name="T">The blob asset's root data type.</typeparam>
        /// <seealso cref="StreamBinaryWriter"/>
        /// <seealso cref="MemoryBinaryWriter"/>
        public static unsafe void Write<T>(this BinaryWriter binaryWriter, BlobAssetReference<T> blob) where T : unmanaged
        {
            var blobAssetLength = blob.m_data.Header->Length;
            var serializeReadyHeader = BlobAssetHeader.CreateForSerialize(blobAssetLength, blob.m_data.Header->Hash);

            binaryWriter.WriteBytes(&serializeReadyHeader, sizeof(BlobAssetHeader));
            binaryWriter.WriteBytes(blob.m_data.Header + 1, blobAssetLength);
        }

        /// <summary>
        /// Reads bytes from a <see cref="BinaryReader"/> instance and deserializes them into a new blob asset.
        /// </summary>
        /// <param name="binaryReader">An implementation of the BinaryReader interface.</param>
        /// <typeparam name="T">The blob asset's root data type.</typeparam>
        /// <returns>A reference to the deserialized blob asset.</returns>
        /// <seealso cref="StreamBinaryReader"/>
        /// <seealso cref="MemoryBinaryReader"/>
        public static unsafe BlobAssetReference<T> Read<T>(this BinaryReader binaryReader) where T : unmanaged
        {
            BlobAssetHeader header;
            binaryReader.ReadBytes(&header, sizeof(BlobAssetHeader));

            var buffer = (byte*)Memory.Unmanaged.Allocate(sizeof(BlobAssetHeader) + header.Length, 16, Allocator.Persistent);
            binaryReader.ReadBytes(buffer + sizeof(BlobAssetHeader), header.Length);

            var bufferHeader = (BlobAssetHeader*)buffer;
            bufferHeader->Allocator = Allocator.Persistent;
            bufferHeader->Length = header.Length;
            bufferHeader->ValidationPtr = buffer + sizeof(BlobAssetHeader);

            // @TODO use 64bit hash
            bufferHeader->Hash = header.Hash;

            BlobAssetReference<T> blobAssetReference;
            blobAssetReference.m_data.m_Align8Union = 0;
            blobAssetReference.m_data.m_Ptr = buffer + sizeof(BlobAssetHeader);

            return blobAssetReference;
        }
    }

    /// <summary>
    /// An untyped reference to a blob assets. UnsafeUntypedBlobAssetReference can be cast to specific typed BlobAssetReferences.
    /// </summary>
    public struct UnsafeUntypedBlobAssetReference : IDisposable, IEquatable<UnsafeUntypedBlobAssetReference>
    {
        internal BlobAssetReferenceData m_data;

        /// <summary>
        /// Creates an unsafe untyped blob asset reference from a BlobAssetReference.
        /// </summary>
        /// <typeparam name="T">The unsafe type.</typeparam>
        /// <param name="blob">Reference to the blob asset that will be referenced by the new UnsafeUntypedBlobAssetReference.</param>
        /// <returns>An unsafe untyped blob asset reference to the blob asset.</returns>
        public static UnsafeUntypedBlobAssetReference Create<T>(BlobAssetReference<T> blob) where T : unmanaged
        {
            UnsafeUntypedBlobAssetReference value;
            value.m_data = blob.m_data;
            return value;
        }

        /// <summary>
        /// Returns a typed blob asset reference to the blob asset referenced by this instance.
        /// </summary>
        /// <typeparam name="T">The reinterpreted type.</typeparam>
        /// <returns>A blob asset reference of the reinterpreted type.</returns>
        public BlobAssetReference<T> Reinterpret<T>() where T : unmanaged
        {
            BlobAssetReference<T> value;
            value.m_data = m_data;
            return value;
        }

        /// <summary>
        /// Disposes the UnsafeUntypedBlobAssetReference object.
        /// </summary>
        public void Dispose()
        {
            m_data.Dispose();
        }

        /// <summary>
        /// Two UnsafeUntypedBlobAssetReference are equal when they reference the same data.
        /// </summary>
        /// <param name="other">The reference to compare to this one.</param>
        /// <returns>True, if both references point to the same data or if both are null.</returns>
        public bool Equals(UnsafeUntypedBlobAssetReference other)
        {
            return m_data.Equals(other.m_data);
        }
    }

    internal unsafe class StreamBinaryReader : BinaryReader
    {
        internal string FilePath { get; }
#if UNITY_EDITOR
        private Stream stream;
        private byte[] buffer;
        public long Position
        {
            get => stream.Position;
            set => stream.Position = value;
        }
#else
        public long Position { get; set; }
#endif
        public StreamBinaryReader(string filePath, long bufferSize = 65536)
        {
            if (string.IsNullOrEmpty(filePath))
                throw new ArgumentException("The filepath can neither be null nor empty", nameof(filePath));

            FilePath = filePath;
#if UNITY_EDITOR
            stream = File.Open(filePath, FileMode.Open, FileAccess.Read, FileShare.Read);
            buffer = new byte[bufferSize];
#else
            Position = 0;
#endif
        }

        public void Dispose()
        {
#if UNITY_EDITOR
            stream.Dispose();
#endif
        }

        public void ReadBytes(void* data, int bytes)
        {
#if UNITY_EDITOR
            int remaining = bytes;
            int bufferSize = buffer.Length;

            fixed (byte* fixedBuffer = buffer)
            {
                while (remaining != 0)
                {
                    int read = stream.Read(buffer, 0, Math.Min(remaining, bufferSize));
                    remaining -= read;
                    UnsafeUtility.MemCpy(data, fixedBuffer, read);
                    data = (byte*)data + read;
                }
            }
#else
            var readCmd = new ReadCommand
            {
                Size = bytes, Offset = Position, Buffer = data
            };
            Assert.IsFalse(string.IsNullOrEmpty(FilePath));
#if ENABLE_PROFILER
            // When AsyncReadManagerMetrics are available, mark up the file read for more informative IO metrics.
            // Metrics can be retrieved by AsyncReadManagerMetrics.GetMetrics
            var readHandle = AsyncReadManager.Read(FilePath, &readCmd, 1, subsystem: AssetLoadingSubsystem.EntitiesStreamBinaryReader);
#else
            var readHandle = AsyncReadManager.Read(FilePath, &readCmd, 1);
#endif
            readHandle.JobHandle.Complete();

            if (readHandle.Status != ReadStatus.Complete)
            {
                throw new IOException($"Failed to read from {FilePath}!");
            }
            Position += bytes;
#endif
        }
    }

    internal unsafe class StreamBinaryWriter : BinaryWriter
    {
        private Stream stream;
        private byte[] buffer;
        public long Position
        {
            get => stream.Position;
            set => stream.Position = value;
        }

        public StreamBinaryWriter(string fileName, int bufferSize = 65536)
        {
            stream = File.Open(fileName, FileMode.Create, FileAccess.Write);
            buffer = new byte[bufferSize];
        }

        public void Dispose()
        {
            stream.Dispose();
        }

        public void WriteBytes(void* data, int bytes)
        {
            int remaining = bytes;
            int bufferSize = buffer.Length;

            fixed (byte* fixedBuffer = buffer)
            {
                while (remaining != 0)
                {
                    int bytesToWrite = Math.Min(remaining, bufferSize);
                    UnsafeUtility.MemCpy(fixedBuffer, data, bytesToWrite);
                    stream.Write(buffer, 0, bytesToWrite);
                    data = (byte*)data + bytesToWrite;
                    remaining -= bytesToWrite;
                }
            }
        }

        public long Length => stream.Length;
    }

    /// <summary>
    /// An interface that writes primitive types to a binary buffer.
    /// </summary>
    /// <seealso cref="MemoryBinaryWriter"/>
    public interface BinaryWriter : IDisposable
    {
        /// <summary>
        /// Writes the specified number of bytes.
        /// </summary>
        /// <param name="data">The data to be written.</param>
        /// <param name="bytes">The number of bytes to write.</param>
        unsafe void WriteBytes(void* data, int bytes);

        /// <summary>
        /// Gets or sets the current write position of the BinaryWriter.
        /// </summary>
        long Position { get; set; }
    }

    /// <summary>
    /// Provides write methods for a BinaryWriter.
    /// </summary>
    public static unsafe class BinaryWriterExtensions
    {
        /// <summary>
        /// Writes a single byte.
        /// </summary>
        /// <param name="writer">The BinaryReader to write to.</param>
        /// <param name="value">The data to write.</param>
        public static void Write(this BinaryWriter writer, byte value)
        {
            writer.WriteBytes(&value, 1);
        }

        /// <summary>
        /// Writes a single int.
        /// </summary>
        /// <param name="writer">The BinaryReader to write to.</param>
        /// <param name="value">The data to write.</param>
        public static void Write(this BinaryWriter writer, int value)
        {
            writer.WriteBytes(&value, sizeof(int));
        }

        /// <summary>
        /// Writes a single ulong.
        /// </summary>
        /// <param name="writer">The BinaryReader to write to.</param>
        /// <param name="value">The data to write.</param>
        public static void Write(this BinaryWriter writer, ulong value)
        {
            writer.WriteBytes(&value, sizeof(ulong));
        }

        /// <summary>
        /// Writes a byte array.
        /// </summary>
        /// <param name="writer">The BinaryReader to write to.</param>
        /// <param name="bytes">The data to write.</param>
        public static void Write(this BinaryWriter writer, byte[] bytes)
        {
            fixed (byte* p = bytes)
            {
                writer.WriteBytes(p, bytes.Length);
            }
        }

        /// <summary>
        /// Writes data from a native array.
        /// </summary>
        /// <param name="writer">The BinaryReader to write to.</param>
        /// <param name="data">The data to write.</param>
        /// <typeparam name="T">The type of data to write from the native array.</typeparam>
        public static void WriteArray<T>(this BinaryWriter writer, NativeArray<T> data) where T : struct
        {
            writer.WriteBytes(data.GetUnsafeReadOnlyPtr(), data.Length * UnsafeUtility.SizeOf<T>());
        }

        /// <summary>
        /// Writes data from a native list.
        /// </summary>
        /// <param name="writer">The BinaryReader to write to.</param>
        /// <param name="data">The data to write.</param>
        /// <typeparam name="T">The type of data to write from the native list.</typeparam>
        public static void WriteList<T>(this BinaryWriter writer, NativeList<T> data) where T : unmanaged
        {
            writer.WriteBytes(data.GetUnsafePtr(), data.Length * UnsafeUtility.SizeOf<T>());
        }

        /// <summary>
        /// Writes the specified number of elements from a native list.
        /// </summary>
        /// <param name="writer">The BinaryReader to write to.</param>
        /// <param name="data">The data to write.</param>
        /// <param name="index">The index at which to start writing from.</param>
        /// <param name="count">The number of elements to write.</param>
        /// <typeparam name="T">The type of data to write from the native list.</typeparam>
        /// <exception cref="ArgumentException">Throws if the index is outside of the buffer range.</exception>
        public static void WriteList<T>(this BinaryWriter writer, NativeList<T> data, int index, int count) where T : unmanaged
        {
            if (index + count > data.Length)
            {
                throw new ArgumentException("index + count must not go beyond the end of the list");
            }
            var size = UnsafeUtility.SizeOf<T>();
            writer.WriteBytes((byte*)data.GetUnsafePtr() + size * index, count * size);
        }
    }

    /// <summary>
    /// An interface that reads primitive types from a binary buffer.
    /// </summary>
    /// <seealso cref="MemoryBinaryReader"/>
    public interface BinaryReader : IDisposable
    {
        /// <summary>
        /// Reads the specified number of bytes.
        /// </summary>
        /// <param name="data">The read data.</param>
        /// <param name="bytes">The number of bytes to read.</param>
        unsafe void ReadBytes(void* data, int bytes);

        /// <summary>
        /// Gets or sets the current read position of the BinaryReader.
        /// </summary>
        long Position { get; set; }
    }

    /// <summary>
    /// Provides additional read methods for the BinaryReader.
    /// </summary>
    public static unsafe class BinaryReaderExtensions
    {
        /// <summary>
        /// Reads a single byte.
        /// </summary>
        /// <param name="reader">The BinaryReader to read from.</param>
        /// <returns>The read data</returns>
        public static byte ReadByte(this BinaryReader reader)
        {
            byte value;
            reader.ReadBytes(&value, 1);
            return value;
        }

        /// <summary>
        /// Reads a single int.
        /// </summary>
        /// <param name="reader">The BinaryReader to read from.</param>
        /// <returns>The read data.</returns>
        public static int ReadInt(this BinaryReader reader)
        {
            int value;
            reader.ReadBytes(&value, sizeof(int));
            return value;
        }

        /// <summary>
        /// Reads a single ulong.
        /// </summary>
        /// <param name="reader">The BinaryReader to read from.</param>
        /// <returns>The read data.</returns>
        public static ulong ReadULong(this BinaryReader reader)
        {
            ulong value;
            reader.ReadBytes(&value, sizeof(ulong));
            return value;
        }

        /// <summary>
        /// Reads the specified number of elements from a native byte array.
        /// </summary>
        /// <param name="reader">The BinaryReader to read from.</param>
        /// <param name="elements">The native array to read of.</param>
        /// <param name="count">The number of elements to read.</param>
        /// <param name="offset">The offset at which to start reading.</param>
        public static void ReadBytes(this BinaryReader reader, NativeArray<byte> elements, int count, int offset = 0)
        {
            byte* destination = (byte*)elements.GetUnsafePtr() + offset;
            reader.ReadBytes(destination, count);
        }

        /// <summary>
        /// Reads the specified number of elements from a native array.
        /// </summary>
        /// <param name="reader">The BinaryReader to read from.</param>
        /// <param name="elements">The native array to read of.</param>
        /// <param name="count">The number of elements to read.</param>
        /// <typeparam name="T">The type of the elements in the native array.</typeparam>
        public static void ReadArray<T>(this BinaryReader reader, NativeArray<T> elements, int count) where T : struct
        {
            reader.ReadBytes((byte*)elements.GetUnsafeReadOnlyPtr(), count * UnsafeUtility.SizeOf<T>());
        }
    }

    /// <summary>
    /// Used by the <see cref="BlobBuilder"/> methods to reference the arrays within a blob asset.
    /// </summary>
    /// <remarks>Use this reference to initialize the data of a newly created <see cref="BlobArray{T}"/>.</remarks>
    /// <typeparam name="T">The data type of the elements in the array.</typeparam>
    public unsafe ref struct BlobBuilderArray<T> where T : struct
    {
        private void* m_data;
        private int m_length;

        /// <summary>
        /// For internal, <see cref="BlobBuilder"/>, use only.
        /// </summary>
        /// <param name="data">The pointer to the allocated buffer.</param>
        /// <param name="length">The length of the buffer.</param>
        public BlobBuilderArray(void* data, int length)
        {
            m_data = data;
            m_length = length;
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        private void CheckIndexOutOfRange(int index)
        {
            if (0 > index || index >= m_length)
                throw new IndexOutOfRangeException(string.Format("Index {0} is out of range of '{1}' Length.", (object)index, (object)this.m_length));
        }

        /// <summary>
        /// Array index accessor for the elements in the array.
        /// </summary>
        /// <param name="index">The sequential index of an array item.</param>
        /// <exception cref="IndexOutOfRangeException">Thrown when index is less than zero or greater than the length of the array (minus one).</exception>
        public ref T this[int index]
        {
            get
            {
                CheckIndexOutOfRange(index);
                return ref UnsafeUtility.ArrayElementAsRef<T>(m_data, index);
            }
        }

        /// <summary>
        /// Reports the number of elements in the array.
        /// </summary>
        public int Length
        {
            get { return m_length; }
        }

        /// <summary>
        /// Provides a pointer to the data stored in the array.
        /// </summary>
        /// <remarks>You can only call this function in an [unsafe context].
        /// [unsafe context]: https://docs.microsoft.com/en-us/dotnet/csharp/language-reference/language-specification/unsafe-code
        /// </remarks>
        /// <returns>A pointer to the first element in the array.</returns>
        public void* GetUnsafePtr()
        {
            return m_data;
        }
    }

    /// <summary>
    /// Creates blob assets.
    /// </summary>
    /// <remarks>
    /// A blob asset is an immutable data structure stored in unmanaged memory.
    /// Blob assets can contain primitive types, strings, structs, arrays, and arrays of arrays. Arrays and structs
    /// must only contain blittable types. Strings must be of type <see cref="BlobString"/> (or a specialized unmanaged
    /// string type).
    ///
    /// To use a BlobBuilder object to create a blob asset:
    /// 1. Declare the structure of the blob asset as a struct.
    /// 2. Create a BlobBuilder object.
    /// 3. Call the <see cref="ConstructRoot{T}"/> method, where `T` is the struct defining the asset structure.
    /// 4. Initialize primitive values defined at the root level of the asset.
    /// 5. Allocate memory for arrays, structs, and <see cref="BlobString"/> instances at the root.
    /// 6. Initialize the values of those arrays, structs, and strings.
    /// 7. Continue allocating memory and initializing values until you have fully constructed the asset.
    /// 8. Call <see cref="CreateBlobAssetReference{T}"/> to create a reference to the blob asset in memory.
    /// 9. Dispose the BlobBuilder object.
    ///
    /// Use the <see cref="BlobAssetReference{T}"/> returned by <see cref="CreateBlobAssetReference{T}"/> to reference
    /// the blob asset. You can use a <see cref="BlobAssetReference{T}"/> as a field of an <see cref="IComponentData"/>
    /// struct. More than one entity can reference the same blob asset.
    ///
    /// Call <see cref="BlobAssetReference{T}.Dispose()"/> to free the memory allocated for a blob asset.
    ///
    /// Blob assets cannot be modified once created. Instead, you must create a new blob asset, update any references
    /// to the old one and then dispose of it.
    /// </remarks>
    /// <example>
    /// <code source="../DocCodeSamples.Tests/BlobAssetExamples.cs" region="builderclassexample" title="BlobBuilder Example"/>
    /// </example>
    unsafe public partial struct BlobBuilder : IDisposable
    {
        AllocatorManager.AllocatorHandle m_allocator;
        NativeList<BlobAllocation> m_allocations;
        NativeList<OffsetPtrPatch> m_patches;
        int m_currentChunkIndex;
        int m_chunkSize;

        struct BlobAllocation
        {
            public int size;
            public byte* p;
        }

        struct BlobDataRef
        {
            public int allocIndex;
            public int offset;
        }

        struct OffsetPtrPatch
        {
            public int* offsetPtr;
            public BlobDataRef target;
            public int length; // if length != 0 this is an array patch and the length should be patched
        }

        /// <summary>
        /// Constructs a BlobBuilder object.
        /// </summary>
        /// <param name="allocator">The type of allocator to use for the BlobBuilder's internal, temporary data. Use
        /// <see cref="Unity.Collections.Allocator.Temp"/> unless the BlobBuilder exists across more than four Unity frames.</param>
        /// <param name="chunkSize">(Optional) The minimum amount of memory to allocate while building an asset.
        /// The default value should suit most use cases. A smaller chunkSize results in more allocations; a larger
        /// chunkSize could increase the BlobBuilder's total memory allocation (which is freed when you dispose of
        /// the BlobBuilder.</param>
        public BlobBuilder(AllocatorManager.AllocatorHandle allocator, int chunkSize = 65536)
        {
            m_allocator = allocator;
            m_allocations = new NativeList<BlobAllocation>(16, m_allocator);
            m_patches = new NativeList<OffsetPtrPatch>(16, m_allocator);
            m_chunkSize = CollectionHelper.Align(chunkSize, 16);
            m_currentChunkIndex = -1;
        }

        /// <summary>
        /// Creates the top-level fields of a single blob asset.
        /// </summary>
        /// <remarks>
        /// This function allocates memory for the top-level fields of a blob asset and returns a reference to it. Use
        /// this root reference to initialize field values and to allocate memory for arrays and structs.
        /// </remarks>
        /// <typeparam name="T">A struct that defines the structure of the blob asset.</typeparam>
        /// <returns>A reference to the blob data under construction.</returns>
        public ref T ConstructRoot<T>() where T : struct
        {
            var allocation = Allocate(UnsafeUtility.SizeOf<T>(), UnsafeUtility.AlignOf<T>());
            return ref UnsafeUtility.AsRef<T>(AllocationToPointer(allocation));
        }

        /// <summary>
        /// Copies an array of structs to an array in a blob asset after allocating the necessary memory.
        /// </summary>
        /// <param name="blobArray">A reference to a BlobArray field in a blob asset.</param>
        /// <param name="data">An array  containing structs of type <typeparamref name="T"/>.</param>
        /// <typeparam name="T">The struct data type.</typeparam>
        /// <returns>A reference to the newly constructed array as a mutable BlobBuilderArray instance.</returns>
        public BlobBuilderArray<T> Construct<T>(ref BlobArray<T> blobArray, params T[] data) where T : struct
        {
            var constructBlobArray = Allocate(ref blobArray, data.Length);
            for (int i = 0; i != data.Length; i++)
                constructBlobArray[i] = data[i];
            return constructBlobArray;
        }

        /// <summary>
        /// Allocates enough memory to store <paramref name="length"/> elements of struct <typeparamref name="T"/>.
        /// </summary>
        /// <param name="ptr">A reference to a BlobArray field in a blob asset.</param>
        /// <param name="length">The number of elements to allocate.</param>
        /// <typeparam name="T">The struct data type.</typeparam>
        /// <returns>A reference to the newly allocated array as a mutable BlobBuilderArray instance.</returns>
        public BlobBuilderArray<T> Allocate<T>(ref BlobArray<T> ptr, int length) where T : struct
        {
            return Allocate(ref ptr, length, UnsafeUtility.AlignOf<T>());
        }

        /// <summary>
        /// Allocates enough memory to store <paramref name="length"/> elements of struct <typeparamref name="T"/>.
        /// </summary>
        /// <param name="ptr">A reference to a BlobArray field in a blob asset.</param>
        /// <param name="length">The number of elements to allocate.</param>
        /// <param name="alignment">The alignment of the allocated memory.</param>
        /// <typeparam name="T">The struct data type.</typeparam>
        /// <returns>A reference to the newly allocated array as a mutable BlobBuilderArray instance.</returns>
        public BlobBuilderArray<T> Allocate<T>(ref BlobArray<T> ptr, int length, int alignment) where T : struct
        {
            if (length <= 0)
                return new BlobBuilderArray<T>(null, 0);

            if (!math.ispow2(alignment))
                throw new ArgumentException("Alignment must be a power of two");

            if (alignment > 16)
                throw new ArgumentException("Alignment larger than 16 is not supported");

            var offsetPtr = (int*)UnsafeUtility.AddressOf(ref ptr.m_OffsetPtr);

            ValidateAllocation(offsetPtr);

            var allocation = Allocate(UnsafeUtility.SizeOf<T>() * length, alignment);

            var patch = new OffsetPtrPatch
            {
                offsetPtr = offsetPtr,
                target = allocation,
                length = length
            };

            m_patches.Add(patch);
            return new BlobBuilderArray<T>(AllocationToPointer(allocation), length);
        }

        /// <summary>
        /// Allocates enough memory to store a struct of type <typeparamref name="T"/>.
        /// </summary>
        /// <param name="ptr">A reference to a blob pointer field in a blob asset.</param>
        /// <typeparam name="T">The struct data type.</typeparam>
        /// <returns>A reference to the newly allocated struct.</returns>
        public ref T Allocate<T>(ref BlobPtr<T> ptr) where T : struct
        {
            var offsetPtr = (int*)UnsafeUtility.AddressOf(ref ptr.m_OffsetPtr);

            ValidateAllocation(offsetPtr);

            var allocation = Allocate(UnsafeUtility.SizeOf<T>(), UnsafeUtility.AlignOf<T>());

            var patch = new OffsetPtrPatch
            {
                offsetPtr = offsetPtr,
                target = allocation,
                length = 0
            };

            m_patches.Add(patch);
            return ref UnsafeUtility.AsRef<T>(AllocationToPointer(allocation));
        }

        /// <summary>
        /// Sets a BlobPtr to point to the given object inside the blob.
        /// </summary>
        /// <param name="ptr">A reference to a blob pointer field in a blob asset.</param>
        /// <param name="obj">The struct that exists in the blob that you want to point to.</param>
        /// <typeparam name="T">The type of the object stored in the blob.</typeparam>
        /// <returns>A reference to obj.</returns>
        /// <exception cref="ArgumentException">Throws if the object is not part of the blob.</exception>
        public ref T SetPointer<T>(ref BlobPtr<T> ptr, ref T obj) where T : struct
        {
            var offsetPtr = (int*)UnsafeUtility.AddressOf(ref ptr.m_OffsetPtr);
            var objTargetPtr = UnsafeUtility.AddressOf(ref obj);

            ValidateAllocation(offsetPtr);

            if (GetPatchTarget(objTargetPtr, out var target))
            {
                var patch = new OffsetPtrPatch
                {
                    offsetPtr = offsetPtr,
                    target = target,
                    length = 0
                };

                m_patches.Add(patch);
            }
            else
            {
                throw new System.ArgumentException("Reference object is not a part of the blob; can only create BlobPtr to objects inside a blob");
            }

            return ref obj;
        }


        struct SortedIndex : IComparable<SortedIndex>
        {
            public byte* p;
            public int index;
            public int CompareTo(SortedIndex other)
            {
                return ((ulong)p).CompareTo((ulong)other.p);
            }
        }

        /// <summary>
        /// Completes construction of the blob asset and returns a reference to the asset in unmanaged memory.
        /// </summary>
        /// <remarks>Use the <see cref="BlobAssetReference{T}"/> to access the blob asset. When the asset is no longer
        /// needed, call<see cref="BlobAssetReference{T}.Dispose()"/> to destroy the blob asset and free its allocated
        /// memory.</remarks>
        /// <param name="allocator">The type of memory to allocate. Unless the asset has a very short life span, use
        /// <see cref="Allocator.Persistent"/>.</param>
        /// <typeparam name="T">The data type of the struct used to construct the asset's root. Use the same struct type
        /// that you used when calling <see cref="ConstructRoot{T}"/>.</typeparam>
        /// <returns>Returns a reference to the blob asset in unmanaged memory.</returns>
        public BlobAssetReference<T> CreateBlobAssetReference<T>(AllocatorManager.AllocatorHandle allocator) where T : unmanaged
        {
            // Avoid crash when there are no chunks (DOTS-8681)
            if (m_currentChunkIndex != -1)
            {
                //Align last chunk upwards so all chunks are 16 byte aligned
                AlignChunk(m_currentChunkIndex);
            }

            var offsets = new NativeArray<int>(m_allocations.Length + 1, Allocator.Temp);
            var sortedAllocs = new NativeArray<SortedIndex>(m_allocations.Length, Allocator.Temp);

            offsets[0] = 0;
            for (int i = 0; i < m_allocations.Length; ++i)
            {
                offsets[i + 1] = offsets[i] + m_allocations[i].size;
                sortedAllocs[i] = new SortedIndex { p = m_allocations[i].p, index = i };
            }
            int dataSize = offsets[m_allocations.Length];

            sortedAllocs.Sort();
            var sortedPatches = new NativeArray<SortedIndex>(m_patches.Length, Allocator.Temp);
            for (int i = 0; i < m_patches.Length; ++i)
                sortedPatches[i] = new SortedIndex { p = (byte*)m_patches[i].offsetPtr, index = i };
            sortedPatches.Sort();

            byte* buffer = (byte*)Memory.Unmanaged.Allocate(sizeof(BlobAssetHeader) + dataSize, 16, allocator);
            byte* data = buffer + sizeof(BlobAssetHeader);

            for (int i = 0; i < m_allocations.Length; ++i)
                UnsafeUtility.MemCpy(data + offsets[i], m_allocations[i].p, m_allocations[i].size);

            int iAlloc = 0;
            var allocStart = m_allocations[sortedAllocs[0].index].p;
            var allocEnd = allocStart + m_allocations[sortedAllocs[0].index].size;

            for (int i = 0; i < m_patches.Length; ++i)
            {
                int patchIndex = sortedPatches[i].index;
                int* offsetPtr = (int*)sortedPatches[i].p;

                while (offsetPtr >= allocEnd)
                {
                    ++iAlloc;
                    allocStart = m_allocations[sortedAllocs[iAlloc].index].p;
                    allocEnd = allocStart + m_allocations[sortedAllocs[iAlloc].index].size;
                }

                var patch = m_patches[patchIndex];

                int offsetPtrInData = offsets[sortedAllocs[iAlloc].index] + (int)((byte*)offsetPtr - allocStart);
                int targetPtrInData = offsets[patch.target.allocIndex] + patch.target.offset;

                *(int*)(data + offsetPtrInData) = targetPtrInData - offsetPtrInData;
                if (patch.length != 0)
                {
                    *(int*)(data + offsetPtrInData + 4) = patch.length;
                }
            }

            sortedPatches.Dispose();
            sortedAllocs.Dispose();
            offsets.Dispose();

            BlobAssetHeader* header = (BlobAssetHeader*)buffer;
            *header = new BlobAssetHeader();
            header->Length = (int)dataSize;
            header->Allocator = allocator;

            // @TODO use 64bit hash
            header->Hash = math.hash(buffer + sizeof(BlobAssetHeader), dataSize);

            BlobAssetReference<T> blobAssetReference;
            blobAssetReference.m_data.m_Align8Union = 0;
            header->ValidationPtr = blobAssetReference.m_data.m_Ptr = buffer + sizeof(BlobAssetHeader);

            return blobAssetReference;
        }

        void* AllocationToPointer(BlobDataRef blobDataRef)
        {
            return m_allocations[blobDataRef.allocIndex].p + blobDataRef.offset;
        }

        BlobAllocation EnsureEnoughRoomInChunk(int size, int alignment)
        {
            if (m_currentChunkIndex == -1)
                return AllocateNewChunk();

            var alloc = m_allocations[m_currentChunkIndex];
            int startOffset = CollectionHelper.Align(alloc.size, alignment);
            if (startOffset + size > m_chunkSize)
                return AllocateNewChunk();

            UnsafeUtility.MemClear(alloc.p + alloc.size, startOffset - alloc.size);

            alloc.size = startOffset;
            return alloc;
        }

        BlobDataRef Allocate(int size, int alignment)
        {
            if (size > m_chunkSize)
            {
                size = CollectionHelper.Align(size, 16);
                var allocIndex = m_allocations.Length;
                var mem = (byte*)Memory.Unmanaged.Allocate(size, alignment, m_allocator);
                UnsafeUtility.MemClear(mem, size);
                m_allocations.Add(new BlobAllocation { p = mem, size = size });
                return new BlobDataRef { allocIndex = allocIndex, offset = 0 };
            }

            BlobAllocation alloc = EnsureEnoughRoomInChunk(size, alignment);

            var offset = alloc.size;
            UnsafeUtility.MemClear(alloc.p + alloc.size, size);
            alloc.size += size;
            m_allocations[m_currentChunkIndex] = alloc;
            return new BlobDataRef { allocIndex = m_currentChunkIndex, offset = offset };
        }

        BlobAllocation AllocateNewChunk()
        {
            // align size of last chunk to 16 bytes so chunks can be concatenated without breaking alignment
            if (m_currentChunkIndex != -1)
            {
                AlignChunk(m_currentChunkIndex);
            }

            m_currentChunkIndex = m_allocations.Length;
            var alloc = new BlobAllocation { p = (byte*)Memory.Unmanaged.Allocate(m_chunkSize, 16, m_allocator), size = 0 };
            m_allocations.Add(alloc);
            return alloc;
        }

        [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS"), Conditional("UNITY_DOTS_DEBUG")]
        void ValidateAllocation(void* address)
        {
            // ValidateAllocation is most often called with data in recently allocated allocations
            // so this searches backwards
            for (int i = m_allocations.Length - 1; i >= 0; --i)
            {
                var allocation = m_allocations[i];
                if (address >= allocation.p && address < (allocation.p + allocation.size))
                    return;
            }

            throw new InvalidOperationException("The BlobArray passed to Allocate was not allocated by this BlobBuilder or the struct that embeds it was copied by value instead of by ref.");
        }

        private bool GetPatchTarget(void* address, out BlobDataRef blobDataRef)
        {
            // Search backwards; most likely to be referring to objects that were most recently added..
            for (int i = m_allocations.Length - 1; i >= 0; i--)
            {
                var allocation = m_allocations[i];
                if (address >= allocation.p && address < (allocation.p + allocation.size))
                {
                    blobDataRef = new BlobDataRef
                    {
                        allocIndex = i,
                        offset = (int)((byte*)address - allocation.p),
                    };

                    return true;
                }
            }

            blobDataRef = default;
            return false;
        }

        /// <summary>
        /// Returns true if this BlobBuilder has been allocated.
        /// </summary>
        public readonly bool IsCreated
            => m_allocations.IsCreated;

        /// <summary>
        /// Disposes of this BlobBuilder instance and frees its temporary memory allocations.
        /// </summary>
        /// <remarks>Call `Dispose()` after calling <see cref="CreateBlobAssetReference{T}"/>.</remarks>
        public void Dispose()
        {
            for (int i = 0; i < m_allocations.Length; ++i)
                Memory.Unmanaged.Free(m_allocations[i].p, m_allocator);
            m_allocations.Dispose();
            m_patches.Dispose();
        }

        void AlignChunk(int chunkIndex)
        {
            var chunk = m_allocations[chunkIndex];
            var oldSize = chunk.size;
            chunk.size = CollectionHelper.Align(chunk.size, 16);
            m_allocations[chunkIndex] = chunk;
            UnsafeUtility.MemSet(chunk.p + oldSize, 0, chunk.size - oldSize);
        }
    }
}