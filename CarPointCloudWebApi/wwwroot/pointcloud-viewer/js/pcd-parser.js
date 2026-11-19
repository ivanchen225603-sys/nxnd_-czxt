/**
 * PCD文件解析器
 * 支持ASCII和binary格式的PCD文件解析
 */

class PCDParser {
    constructor() {
        this.header = {};
        this.dataOffset = 0;
    }

    /**
     * 解析PCD文件内容
     * @param {string|ArrayBuffer} content - PCD文件内容（文本或二进制）
     * @returns {Object} 解析后的点云数据
     */
    parse(content) {
        try {
            // 判断输入类型
            const isBinary = content instanceof ArrayBuffer;
            let text;
            if (isBinary) {
                // 将二进制数据解码为文本以解析文件头
                text = new TextDecoder('latin1').decode(content);
            } else {
                text = content;
            }

            // 解析文件头
            const headerEnd = this.parseHeader(text);

            // 根据数据格式解析点云数据
            let pcdData;
            if (this.header.DATA === 'ascii') {
                pcdData = this.parseAsciiData(text, headerEnd);
            } else if (this.header.DATA === 'binary') {
                if (!isBinary) {
                    throw new Error('binary格式PCD需要使用ArrayBuffer读取');
                }
                pcdData = this.parseBinaryData(content, headerEnd);
            } else if (this.header.DATA === 'binary_compressed') {
                throw new Error('暂不支持compressed格式，请使用ASCII格式');
            } else {
                throw new Error(`未知的数据格式: ${this.header.DATA}`);
            }

            return pcdData;
        } catch (error) {
            throw new Error(`PCD解析失败: ${error.message}`);
        }
    }

    /**
     * 解析PCD文件头
     * @param {string} text - 文件文本
     * @returns {number} 文件头结束位置（数据开始位置）
     */
    parseHeader(text) {
        const lines = text.split('\n');
        let lineIndex = 0;
        
        // 重置header
        this.header = {};
        
        for (let i = 0; i < lines.length; i++) {
            const line = lines[i].trim();
            
            // 跳过空行和注释
            if (!line || line.startsWith('#')) {
                continue;
            }
            
            const parts = line.split(/\s+/);
            const key = parts[0];
            
            switch (key) {
                case 'VERSION':
                    this.header.VERSION = parts[1];
                    break;
                case 'FIELDS':
                    this.header.FIELDS = parts.slice(1);
                    break;
                case 'SIZE':
                    this.header.SIZE = parts.slice(1).map(Number);
                    break;
                case 'TYPE':
                    this.header.TYPE = parts.slice(1);
                    break;
                case 'COUNT':
                    this.header.COUNT = parts.slice(1).map(Number);
                    break;
                case 'WIDTH':
                    this.header.WIDTH = parseInt(parts[1]);
                    break;
                case 'HEIGHT':
                    this.header.HEIGHT = parseInt(parts[1]);
                    break;
                case 'VIEWPOINT':
                    this.header.VIEWPOINT = parts.slice(1).map(Number);
                    break;
                case 'POINTS':
                    this.header.POINTS = parseInt(parts[1]);
                    break;
                case 'DATA':
                    this.header.DATA = parts[1];
                    lineIndex = i + 1;
                    break;
                default:
                    console.warn(`未知的字段: ${key}`);
            }
            
            // 如果遇到DATA字段，文件头结束
            if (key === 'DATA') {
                break;
            }
        }
        
        // 验证必要的字段
        this.validateHeader();
        
        // 计算数据偏移量（字节数）
        const headerText = lines.slice(0, lineIndex).join('\n') + '\n';
        this.dataOffset = headerText.length;
        
        return this.dataOffset;
    }

    /**
     * 验证文件头完整性
     */
    validateHeader() {
        const requiredFields = ['VERSION', 'FIELDS', 'SIZE', 'TYPE', 'COUNT', 
                                'WIDTH', 'HEIGHT', 'VIEWPOINT', 'POINTS', 'DATA'];
        
        for (const field of requiredFields) {
            if (!(field in this.header)) {
                throw new Error(`缺少必要字段: ${field}`);
            }
        }
        
        if (this.header.POINTS <= 0) {
            throw new Error('点数必须大于0');
        }
        
        if (!this.header.FIELDS.includes('x') || 
            !this.header.FIELDS.includes('y') || 
            !this.header.FIELDS.includes('z')) {
            throw new Error('PCD文件必须包含x, y, z字段');
        }
    }

    /**
     * 解析ASCII格式的点云数据
     * @param {string} text - 文件文本
     * @param {number} headerEnd - 文件头结束位置
     * @returns {Object} 点云数据
     */
    parseAsciiData(text, headerEnd) {
        const dataText = text.substring(headerEnd).trim();
        const lines = dataText.split('\n').filter(line => line.trim());
        
        const pointCount = lines.length;
        const positions = new Float32Array(pointCount * 3);
        let intensities = null;
        let hasIntensity = false;
        
        // 检查是否有intensity字段
        if (this.header.FIELDS.includes('intensity')) {
            hasIntensity = true;
            intensities = new Float32Array(pointCount);
        }
        
        const fieldIndices = {
            x: this.header.FIELDS.indexOf('x'),
            y: this.header.FIELDS.indexOf('y'),
            z: this.header.FIELDS.indexOf('z')
        };
        
        if (hasIntensity) {
            fieldIndices.intensity = this.header.FIELDS.indexOf('intensity');
        }
        
        // 解析每一行数据
        for (let i = 0; i < pointCount; i++) {
            const values = lines[i].trim().split(/\s+/).map(Number);
            
            positions[i * 3] = values[fieldIndices.x];
            positions[i * 3 + 1] = values[fieldIndices.y];
            positions[i * 3 + 2] = values[fieldIndices.z];
            
            if (hasIntensity) {
                intensities[i] = values[fieldIndices.intensity];
            }
        }
        
        return {
            positions: positions,
            colors: null,
            intensities: hasIntensity ? intensities : null,
            pointCount: pointCount,
            fields: this.header.FIELDS
        };
    }

    /**
     * 解析binary格式的点云数据
     * @param {ArrayBuffer} buffer - 二进制数据
     * @param {number} headerEnd - 文件头结束位置（字节偏移）
     * @returns {Object} 点云数据
     */
    parseBinaryData(buffer, headerEnd) {
        const pointCount = this.header.POINTS;
        const fields = this.header.FIELDS;
        const sizes = this.header.SIZE;
        const types = this.header.TYPE;
        const counts = this.header.COUNT;

        // 计算每个点占用的字节数
        let pointSize = 0;
        for (let i = 0; i < fields.length; i++) {
            pointSize += sizes[i] * counts[i];
        }

        // 检查数据长度是否匹配
        const expectedDataSize = pointCount * pointSize;
        const actualDataSize = buffer.byteLength - headerEnd;
        if (actualDataSize < expectedDataSize) {
            throw new Error(`数据长度不匹配: 期望 ${expectedDataSize} 字节, 实际 ${actualDataSize} 字节`);
        }

        const dataView = new DataView(buffer, headerEnd);
        const positions = new Float32Array(pointCount * 3);
        let intensities = null;
        let hasIntensity = false;

        // 检查是否有intensity字段
        if (fields.includes('intensity')) {
            hasIntensity = true;
            intensities = new Float32Array(pointCount);
        }

        // 计算每个字段的偏移量
        const fieldOffsets = {};
        let currentOffset = 0;
        for (let i = 0; i < fields.length; i++) {
            fieldOffsets[fields[i]] = currentOffset;
            currentOffset += sizes[i] * counts[i];
        }

        const xOffset = fieldOffsets['x'];
        const yOffset = fieldOffsets['y'];
        const zOffset = fieldOffsets['z'];
        const intensityOffset = hasIntensity ? fieldOffsets['intensity'] : -1;

        // 获取字段的size和type
        const xIndex = fields.indexOf('x');
        const yIndex = fields.indexOf('y');
        const zIndex = fields.indexOf('z');
        const intensityIndex = hasIntensity ? fields.indexOf('intensity') : -1;

        // 解析每个点
        for (let i = 0; i < pointCount; i++) {
            const pointOffset = i * pointSize;

            positions[i * 3] = this.readBinaryValue(dataView, pointOffset + xOffset, sizes[xIndex], types[xIndex]);
            positions[i * 3 + 1] = this.readBinaryValue(dataView, pointOffset + yOffset, sizes[yIndex], types[yIndex]);
            positions[i * 3 + 2] = this.readBinaryValue(dataView, pointOffset + zOffset, sizes[zIndex], types[zIndex]);

            if (hasIntensity) {
                intensities[i] = this.readBinaryValue(dataView, pointOffset + intensityOffset, sizes[intensityIndex], types[intensityIndex]);
            }
        }

        return {
            positions: positions,
            colors: null,
            intensities: hasIntensity ? intensities : null,
            pointCount: pointCount,
            fields: fields
        };
    }

    /**
     * 读取二进制值
     * @param {DataView} dataView - 数据视图
     * @param {number} offset - 字节偏移量
     * @param {number} size - 字段大小（字节）
     * @param {string} type - 字段类型（I, F, U等）
     * @returns {number} 读取的数值
     */
    readBinaryValue(dataView, offset, size, type) {
        switch (type) {
            case 'F': // 浮点数
                if (size === 4) {
                    return dataView.getFloat32(offset, true); // little-endian
                } else if (size === 8) {
                    return dataView.getFloat64(offset, true);
                }
                break;
            case 'I': // 有符号整数
                if (size === 1) {
                    return dataView.getInt8(offset);
                } else if (size === 2) {
                    return dataView.getInt16(offset, true);
                } else if (size === 4) {
                    return dataView.getInt32(offset, true);
                }
                break;
            case 'U': // 无符号整数
                if (size === 1) {
                    return dataView.getUint8(offset);
                } else if (size === 2) {
                    return dataView.getUint16(offset, true);
                } else if (size === 4) {
                    return dataView.getUint32(offset, true);
                }
                break;
        }
        // 默认返回0
        return 0;
    }

    /**
     * 获取坐标范围
     * @param {Float32Array} positions - 位置数组
     * @returns {Object} 坐标范围 {minX, minY, minZ, maxX, maxY, maxZ}
     */
    getBoundingBox(positions) {
        let minX = Infinity, minY = Infinity, minZ = Infinity;
        let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
        
        for (let i = 0; i < positions.length; i += 3) {
            const x = positions[i];
            const y = positions[i + 1];
            const z = positions[i + 2];
            
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (z < minZ) minZ = z;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
            if (z > maxZ) maxZ = z;
        }
        
        return { minX, minY, minZ, maxX, maxY, maxZ };
    }

    /**
     * 计算中心点
     * @param {Float32Array} positions - 位置数组
     * @returns {Object} 中心点 {x, y, z}
     */
    getCenter(positions) {
        const bbox = this.getBoundingBox(positions);
        return {
            x: (bbox.minX + bbox.maxX) / 2,
            y: (bbox.minY + bbox.maxY) / 2,
            z: (bbox.minZ + bbox.maxZ) / 2
        };
    }

    /**
     * 计算包围盒尺寸
     * @param {Float32Array} positions - 位置数组
     * @returns {Object} 尺寸 {width, height, depth}
     */
    getSize(positions) {
        const bbox = this.getBoundingBox(positions);
        return {
            width: bbox.maxX - bbox.minX,
            height: bbox.maxY - bbox.minY,
            depth: bbox.maxZ - bbox.minZ
        };
    }
}
