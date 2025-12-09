public static class GraphUtil
{
    const int halfEdgeHashShift = sizeof(uint) * 4;
    public static uint HalfEdgeHash(int ID_A, int ID_B)
    {
        return ID_A > ID_B ? ((uint)ID_A << halfEdgeHashShift) | (uint)ID_B
                           : ((uint)ID_B << halfEdgeHashShift) | (uint)ID_A;
    }
}